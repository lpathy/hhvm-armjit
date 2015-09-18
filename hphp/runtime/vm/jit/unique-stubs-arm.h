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

#ifndef incl_HPHP_JIT_UNIQUE_STUBS_ARM_H_
#define incl_HPHP_JIT_UNIQUE_STUBS_ARM_H_

#include "hphp/runtime/vm/jit/abi-arm.h"
#include "hphp/runtime/vm/jit/vasm-gen.h"
#include "hphp/runtime/vm/jit/vasm-reg.h"

#include "hphp/util/asm-x64.h"
#include "hphp/util/data-block.h"

namespace HPHP { namespace jit { namespace arm {

///////////////////////////////////////////////////////////////////////////////

/*
 * Arch-dependent helpers needed to emit unique stubs.
 *
 * Mirrors the API at the top of unique-stubs.cpp.
 */

inline RegSet syncForLLVMCatch(Vout& v) { return RegSet(); }

inline void loadSavedRIP(Vout& v, Vreg d) {
  v << load{*PhysReg(rLinkReg), d};
}

inline void stashSavedRIP(Vout& v, Vreg fp) {
  v << store{PhysReg(rLinkReg), fp[AROFF(m_savedRip)]};
}

inline void unstashSavedRIP(Vout& v, Vreg fp) {
  v << load{fp[AROFF(m_savedRip)], PhysReg(rLinkReg)};
}

///////////////////////////////////////////////////////////////////////////////

TCA emitFunctionEnterHelper(CodeBlock& cb, UniqueStubs& us);
TCA emitFreeLocalsHelpers(CodeBlock& cb, UniqueStubs& us);
TCA emitCallToExit(CodeBlock& cb);
TCA emitEndCatchHelper(CodeBlock& cb, UniqueStubs& us);

TCA emitEnterTCHelper(Codeblock& cb, UniqueStubs& us);

///////////////////////////////////////////////////////////////////////////////

}}}

#endif
