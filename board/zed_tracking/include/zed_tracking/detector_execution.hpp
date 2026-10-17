#pragma once
#include <future>
#include <utility>
namespace zed_tracking {
template<class Function> auto launch_secondary(Function&& function,bool parallel){
 return std::async(parallel?std::launch::async:std::launch::deferred,std::forward<Function>(function));
}
}
