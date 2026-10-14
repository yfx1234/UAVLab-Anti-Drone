#include <zed_tracking/detector_execution.hpp>
#include <future>
#include <cassert>
#include <chrono>
#include <stdexcept>
int main(){
 std::promise<void> started,release;auto ready=started.get_future();auto gate=release.get_future().share();
 auto task=zed_tracking::launch_secondary([&]{started.set_value();gate.wait();return 42;},true);
 assert(ready.wait_for(std::chrono::seconds(2))==std::future_status::ready);
 release.set_value();assert(task.get()==42);
 bool ran=false;auto sequential=zed_tracking::launch_secondary([&]{ran=true;return 7;},false);
 assert(sequential.wait_for(std::chrono::milliseconds(0))==std::future_status::deferred);assert(!ran);assert(sequential.get()==7&&ran);
 auto failed=zed_tracking::launch_secondary([]()->int{throw std::runtime_error("secondary failed");},true);
 bool propagated=false;try{failed.get();}catch(const std::runtime_error&){propagated=true;}assert(propagated);
}
