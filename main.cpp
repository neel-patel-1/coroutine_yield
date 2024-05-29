#include <boost/coroutine/all.hpp>

#include <cstdlib>
#include <iostream>
#include <immintrin.h>
#include <vector>

#include <x86intrin.h>

#include <boost/bind.hpp>

#include "dml/dml.hpp"
typedef boost::coroutines::symmetric_coroutine< void >  coro_t;


int num_request_contexts = 10;
int size = 4096;
static constexpr int num_requests = 1024;

int cur_request = 0;
dml::handler<dml::mem_move_operation, std::allocator<std::uint8_t>> handlers[num_requests]; /* I want to have multiple requests in flight and have requests being preempted */

void request_fn( coro_t::yield_type &yield){
  for(int i=0; i<100; i++){}
  int next_submit_idx = cur_request;

  auto src_data = std::vector<uint8_t>('a',size);
  auto dst_data = std::vector<uint8_t>(src_data.size());

  handlers[next_submit_idx] = dml::submit<dml::hardware>(dml::mem_move,
                                        dml::make_view(src_data),
                                        dml::make_view(dst_data));

  cur_request++;
  yield();



  auto result = handlers[next_submit_idx].get();
  if (result.status != dml::status_code::ok)
  {
      std::cout<<"dsa fail\n";
  }

}

void scheduler( coro_t::yield_type & yield)
{
  coro_t::call_type *idle_coroutine;
  coro_t::call_type * c2[num_request_contexts];

  std::cout << "Called Scheduler\n";
  coro_t::call_type request_coro{request_fn};

  /* Make the request contexts */
  #ifdef BREAKDOWN
  uint64_t request_start_time = __rdtsc();
  #endif

  request_coro();

  #ifdef BREAKDOWN
  uint64_t after_yield = __rdtsc();
  #endif

  request_coro();

}

int main( int argc, char * argv[])
{
  coro_t::call_type coro1(scheduler);
  int size = 1024;

  coro1();

    std::cout << "Done" << std::endl;

    return EXIT_SUCCESS;
}