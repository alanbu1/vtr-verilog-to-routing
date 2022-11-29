#pragma once

/* std::optional-like interface with optional references.
 * currently: import TartanLlama's optional into the vtr namespace
 * documentation at https://tl.tartanllama.xyz/en/latest/api/optional.html
 * there are three main uses of this:
 * 1. replace pointers when refactoring legacy code
 *   optional<T&> (reference) is in many ways a pointer, it even has * and -> operators,
 *   but it can't be allocated or freed. this property is very helpful when
 *   refactoring code with a lot of malloc, free, new and delete.
 * 2. explicit alternative for containers
 *   optional<T> (non-reference) allows you to put non-empty-initializable
 *   objects into a container which owns them. it is an alternative to
 *   unique_ptr<T> in that sense, but with a cleaner interface.
 * 3. function return types
 *   returning an optional<T> gives the caller a clear hint to check the return value.
 */

#include "tl_optional.hpp"

namespace vtr {
template<class T>
using optional = tl::optional<T>;

using nullopt_t = tl::nullopt_t;
static constexpr nullopt_t nullopt = tl::nullopt;

using bad_optional_access = tl::bad_optional_access;
} // namespace vtr
