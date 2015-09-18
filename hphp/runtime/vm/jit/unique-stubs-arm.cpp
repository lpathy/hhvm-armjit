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

#include "hphp/runtime/vm/jit/align-arm.h"
#include "hphp/runtime/vm/jit/code-gen-cf.h"
#include "hphp/runtime/vm/jit/unique-stubs-arm.h"

namespace HPHP { namespace jit { namespace arm {

TRACE_SET_MOD(ustubs);

///////////////////////////////////////////////////////////////////////////////

static void alignJmpTarget(CodeBlock& cb) {
  align(cb, Alignment::JmpTarget, AlignContext::Dead);
}

//////////////////////////////////////////////////////////////////////////////

TCA emitFunctionEnterHelper(CodeBlock& cb, UniqueStubs& us) {
  // FIXME: This function is near identical to its X64 equivalent, except for
  // the pushing and popping of the link register, and using lea to manipulate
  // the stack pointer.

  alignJmpTarget(cb);

  auto const start = vwrap(cb, [&] (Vout& v) {
    auto const ar = v.makeReg();

    v << copy{rvmfp(), ar};

    // Set up the call frame for the stub.  We can't skip this like we do in
    // other stubs because we need the return IP for this frame in the %rbp
    // chain, in order to find the proper fixup for the VMRegAnchor in the
    // intercept handler.

    v << push{PhysReg{rLinkReg}};
    v << push{rvmfp()};
    v << copy{rsp(), rvmfp()};

    // When we call the event hook, it might tell us to skip the callee
    // (because of fb_intercept).  If that happens, we need to return to the
    // caller, but the handler will have already popped the callee's frame.
    // So, we need to save these values for later.
    v << pushm{ar[AROFF(m_savedRip)]};
    v << pushm{ar[AROFF(m_sfp)]};

    v << copy2{ar, v.cns(EventHook::NormalFunc), rarg(0), rarg(1)};

    bool (*hook)(const ActRec*, int) = &EventHook::onFunctionCall;
    v << call{TCA(hook), rarg(0)|rarg(1)};
  });

  us.functionEnterHelperReturn = vwrap2(cb, [&] (Vout& v, Vout& vcold) {
    auto const sf = v.makeReg();
    v << testb{rret(), rret(), sf};

    unlikelyIfThen(v, vcold, CC_Z, sf, [&] (Vout& v) {
      auto const saved_rip = v.makeReg();

      // The event hook has already cleaned up the stack and popped the
      // callee's frame, so we're ready to continue from the original call
      // site.  We just need to grab the fp/rip of the original frame that we
      // saved earlier, and sync rvmsp().
      v << pop{rvmfp()};
      v << pop{saved_rip};

      // Drop our call frame.
      v << lea{rsp()[16], rsp()};

      // Sync vmsp and return to the caller.  This unbalances the return stack
      // buffer, but if we're intercepting, we probably don't care.
      v << load{rvmtl()[rds::kVmspOff], rvmsp()};
      v << jmpr{saved_rip};
    });

    // Skip past the stuff we saved for the intercept case.
    v << lea{rsp()[16], rsp()};

    // Execute a leave, returning us to the callee's prologue.
    v << pop{rvmfp()};
    v << pop{PhysReg{rLinkReg}};
    v << ret{};
  });

  return start;
}

TCA emitFreeLocalsHelpers(CodeBlock& cb, UniqueStubs& us) {
  auto const release = vwrap(cb, [&](Vout& v) {
    v << ud2{};
  });
  us.freeManyLocalsHelper = vwrap(cb, [&](Vout& v) {
    v << ud2{};
  });
  return release;
}

TCA emitCallToExit(CodeBlock& cb) {
  return vwrap(cb, [&](Vout& v) {
    // Sync VM regs:
    v << store{rvmsp(), rvmtl()[rds::kVmspOff]};
    v << store{rvmfp(), rvmtl()[rds::kVmfpOff]};
    // Epilogue:
    v << pop{rvmfp()};
    v << pop{PhysReg{rLinkReg}};
    v << ret{};
  });
}

TCA emitEndCatchHelper(CodeBlock& cb, UniqueStubs& us) {
  us.endCatchHelperPast = vwrap(cb, [&](Vout& v) {
    v << ud2{};
  });
  return vwrap(cb, [&](Vout& v) {
    v << ud2{};
  });
}

}}}
