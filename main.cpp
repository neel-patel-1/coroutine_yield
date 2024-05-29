#include <boost/coroutine/all.hpp>

#include <cstdlib>
#include <iostream>
#include <immintrin.h>
#include <vector>

#include <boost/bind.hpp>

#include "dml/dml.hpp"

int num_request_contexts = 10;
int size = 4096;

typedef boost::coroutines::symmetric_coroutine< void >  coro_t;

void request_fn( coro_t::yield_type &yield){
  for(int i=0; i<100; i++){}

  auto src_data = std::vector<uint8_t>('a',size);
  auto dst_data = std::vector<uint8_t>(src_data.size());

  auto handler = dml::submit<dml::hardware>(dml::mem_move,
                                        dml::make_view(src_data),
                                        dml::make_view(dst_data));

  yield();


  auto result = handler.get();
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
  coro_t::call_type request{request_fn};

  /* Make the request contexts */

}

int main( int argc, char * argv[])
{
  coro_t::call_type coro1(scheduler);
  int size = 1024;

  coro1();

    std::cout << "Done" << std::endl;

    return EXIT_SUCCESS;
}