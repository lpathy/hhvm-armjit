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

#ifndef incl_HPHP_JIT_ABI_ARM_H
#define incl_HPHP_JIT_ABI_ARM_H

#include "hphp/runtime/vm/jit/abi-regs.h"
#include "hphp/runtime/vm/jit/phys-reg.h"

#include "hphp/util/asm-x64.h"
#include "hphp/vixl/a64/assembler-a64.h"
#include "hphp/vixl/a64/constants-a64.h"

#include <unordered_map>

namespace HPHP { namespace jit {

struct Abi;

namespace arm {

///////////////////////////////////////////////////////////////////////////////

/*
 * Mirrors the API of abi.h.
 */

const Abi& abi(CodeKind kind = CodeKind::Trace);

inline PhysReg rvmfp() { return vixl::x29; }
inline PhysReg rvmsp() { return vixl::x19; }
inline PhysReg rvmtl() { return vixl::x20; }
inline PhysReg rsp()   { return vixl::sp; }

namespace detail {
  const RegSet kVMRegs      = rvmfp() | rvmtl();
  const RegSet kVMRegsNoSP  = rvmfp() | rvmtl() | rvmsp();
}

inline RegSet vm_regs_with_sp() { return detail::kVMRegs; }
inline RegSet vm_regs_no_sp()   { return detail::kVMRegsNoSP; }

PhysReg rret(size_t i = 0);
PhysReg rret_simd(size_t i);

PhysReg rarg(size_t i);
PhysReg rarg_simd(size_t i);

constexpr size_t num_arg_regs() { return 7; }
constexpr size_t num_arg_regs_simd() { return 0; }

RegSet arg_regs(size_t n);
RegSet arg_regs_simd(size_t n);

PhysReg r_svcreq_req();
PhysReg r_svcreq_stub();
PhysReg r_svcreq_sf();
PhysReg r_svcreq_arg(size_t i);

///////////////////////////////////////////////////////////////////////////////

inline vixl::Register x2a(PhysReg x64reg) {
  always_assert(!x64reg.isSIMD());
  return vixl::Register(vixl::CPURegister(x64reg));
}

inline vixl::FPRegister x2simd(PhysReg x64reg) {
  always_assert(x64reg.isSIMD());
  return vixl::FPRegister(vixl::CPURegister(x64reg));
}

inline vixl::Condition convertCC(jit::ConditionCode cc) {
  if (cc == jit::CC_P || cc == jit::CC_NP) {
    // ARM has no parity flag
    always_assert(false);
  }
  assertx(cc >= 0 && cc <= 0xF);

  using namespace vixl;

  // We'll index into this array by the x64 condition code.
  constexpr vixl::Condition mapping[] = {
    vs,  // overflow set
    vc,  // overflow clear
    lo,  // unsigned lower
    hs,  // unsigned higher or same
    eq,  // equal
    ne,  // not equal
    ls,  // unsigned lower or same
    hi,  // unsigned higher
    pl,  // plus (sign set)
    mi,  // minus (sign clear)
    nv, nv,  // invalid. These are the parity flags.
    lt,  // signed less than
    ge,  // signed greater or equal
    le,  // signed less or equal
    gt,  // signed greater than
  };

  return mapping[cc];
}

inline jit::ConditionCode convertCC(vixl::Condition cc) {
  switch (cc) {
    case vixl::eq: return jit::CC_E;
    case vixl::ne: return jit::CC_NE;
    case vixl::hs: return jit::CC_AE;
    case vixl::lo: return jit::CC_B;
    case vixl::mi: return jit::CC_NS;
    case vixl::pl: return jit::CC_S;
    case vixl::vs: return jit::CC_O;
    case vixl::vc: return jit::CC_NO;
    case vixl::hi: return jit::CC_A;
    case vixl::ls: return jit::CC_NA;
    case vixl::ge: return jit::CC_GE;
    case vixl::lt: return jit::CC_L;
    case vixl::gt: return jit::CC_G;
    case vixl::le: return jit::CC_LE;
    default: not_reached();
  }
}

///////////////////////////////////////////////////////////////////////////////

inline vixl::Register svcReqArgReg(unsigned index) {
  // First arg holds the request number
  return x2a(rarg(index + 1));
}

const vixl::Register rAsm(vixl::x9);
const vixl::Register rAsm2(vixl::x10);
const vixl::Register rGContextReg(vixl::x24);
const vixl::Register rLinkReg(vixl::x30);
const vixl::Register rHostCallReg(vixl::x16);

///////////////////////////////////////////////////////////////////////////////

}}}

#endif
