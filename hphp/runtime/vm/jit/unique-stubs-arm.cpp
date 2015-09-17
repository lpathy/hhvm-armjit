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

#include "hphp/runtime/vm/jit/unique-stubs-arm.h"

namespace HPHP { namespace jit { namespace arm {

TRACE_SET_MOD(ustubs);

TCA emitFunctionEnterHelper(CodeBlock& cb, UniqueStubs& us) {
  auto const start = vwrap(cb, [&](Vout& v) {
    v << ud2{};
  });
  us.functionEnterHelperReturn = vwrap(cb, [&](Vout& v) {
    v << ud2{};
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
    v << ud2{};
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
