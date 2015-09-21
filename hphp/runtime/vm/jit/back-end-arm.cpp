/*
   +----------------------------------------------------------------------+
   | HipHop for PHP                                                       |
   +----------------------------------------------------------------------+
   | Copyright (c) 2010-2015 Facebook, Inc. (http://www.facebook.com)     |
   +----------------------------------------------------------------------+
   | This source file is subject to version 3.01 of the PHP license,      |
   | that is bundled with this package in the file LICENSE, and is        |
   | available through the world-wide-web at the following url:           |
   | http://www.php.net/license/3_01.txt                                  |
   | If you did not receive a copy of the PHP license and are unable to   |
   | obtain it through the world-wide-web, please send a note to          |
   | license@php.net so we can mail you a copy immediately.               |
   +----------------------------------------------------------------------+
*/

#include "hphp/runtime/vm/jit/back-end-arm.h"

#include <iostream>

#include "hphp/vixl/a64/disasm-a64.h"
#include "hphp/vixl/a64/macro-assembler-a64.h"
#include "hphp/util/text-color.h"

#include "hphp/runtime/vm/jit/abi-arm.h"
#include "hphp/runtime/vm/jit/block.h"
#include "hphp/runtime/vm/jit/func-guard-arm.h"
#include "hphp/runtime/vm/jit/mc-generator.h"
#include "hphp/runtime/vm/jit/unwind-arm.h"
#include "hphp/runtime/vm/jit/service-requests.h"
#include "hphp/runtime/vm/jit/timer.h"
#include "hphp/runtime/vm/jit/check.h"
#include "hphp/runtime/vm/jit/print.h"
#include "hphp/runtime/vm/jit/cfg.h"
#include "hphp/runtime/vm/jit/vasm-emit.h"
#include "hphp/runtime/vm/jit/vasm-gen.h"
#include "hphp/runtime/vm/jit/vasm-print.h"

namespace HPHP { namespace jit { namespace arm {

TRACE_SET_MOD(hhir);

namespace {

struct BackEnd final : jit::BackEnd {
  BackEnd() {}
  ~BackEnd() {}

#if defined(__aarch64__)
#define CALLEE_SAVED_BARRIER() \
  asm volatile("" : : : "x19", "x20", "x21", "x22", "x23", "x24", "x25", \
               "x26", "x27", "x28")
#else
#define CALLEE_SAVED_BARRIER()
#endif

  void enterTCHelper(TCA start, ActRec* stashedAR) override {
    if (RuntimeOption::EvalSimulateARM) {
      vixl::Decoder decoder;
      vixl::Simulator sim(&decoder, std::cout);
      if (getenv("ARM_DISASM")) {
        sim.set_disasm_trace(true);
      }
      SCOPE_EXIT {
        Stats::inc(Stats::vixl_SimulatedInstr, sim.instr_count());
        Stats::inc(Stats::vixl_SimulatedLoad, sim.load_count());
        Stats::inc(Stats::vixl_SimulatedStore, sim.store_count());
      };
      sim.set_exception_hook(arm::simulatorExceptionHook);

      g_context->m_activeSims.push_back(&sim);
      SCOPE_EXIT { g_context->m_activeSims.pop_back(); };

      auto& regs = vmRegsUnsafe();
      sim.set_xreg(x2a(rarg(0)).code(), regs.stack.top());
      sim.set_xreg(x2a(rarg(1)).code(), regs.fp);
      sim.set_xreg(x2a(rarg(2)).code(), start);
      sim.set_xreg(x2a(rarg(3)).code(), vmFirstAR());
      sim.set_xreg(x2a(rarg(4)).code(), rds::tl_base);
      sim.set_xreg(x2a(rarg(5)).code(), stashedAR);

      DEBUG_ONLY auto spOnEntry = sim.sp();

      std::cout.flush();
      sim.RunFrom(
        vixl::Instruction::Cast(
          mcg->tx().uniqueStubs.enterTCHelper
        )
      );
      std::cout.flush();

      assertx(sim.sp() == spOnEntry);
    } else {
      // We have to force C++ to spill anything that might be in a callee-saved
      // register. enterTCHelper does not save them.
      CALLEE_SAVED_BARRIER();
      auto& regs = vmRegsUnsafe();
      reinterpret_cast<void(*)(Cell*, ActRec*, TCA, ActRec*, void*, ActRec*)>(
        mcg->tx().uniqueStubs.enterTCHelper
      )(
        regs.stack.top(),
        regs.fp,
        start,
        vmFirstAR(),
        rds::tl_base,
        stashedAR
      );
      CALLEE_SAVED_BARRIER();
    }
  }

  void streamPhysReg(std::ostream& os, PhysReg reg) override {
    if (reg.isSF()) {
      os << "statusFlags";
      return;
    }
    auto prefix = reg.isGP() ? (vixl::Register(reg).size() == vixl::kXRegSize
                                ? 'x' : 'w')
                  : (vixl::FPRegister(reg).size() == vixl::kSRegSize
                     ? 's' : 'd');
    vixl::CPURegister r = reg;
    os << prefix << r.code();
  }

  void disasmRange(std::ostream& os, int indent, bool dumpIR, TCA begin,
                   TCA end) override {
    using namespace vixl;
    Decoder dec;
    PrintDisassembler disasm(os, indent + 4, true, color(ANSI_COLOR_BROWN));
    dec.AppendVisitor(&disasm);
    assertx(begin <= end);
    for (; begin < end; begin += kInstructionSize) {
      dec.Decode(Instruction::Cast(begin));
    }
  }
};

}

std::unique_ptr<jit::BackEnd> newBackEnd() {
  return folly::make_unique<BackEnd>();
}

}}}
