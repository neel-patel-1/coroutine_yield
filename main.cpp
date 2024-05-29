#include <boost/coroutine/all.hpp>

#include <cstdlib>
#include <iostream>
#include <immintrin.h>

#include <boost/bind.hpp>

typedef boost::coroutines::symmetric_coroutine< void >  coro_t;

coro_t::call_type * c1 = 0;

void scheduler( coro_t::yield_type & yield)
{
  std::cout << "Scheduler invoked" << std::endl;
}

int main( int argc, char * argv[])
{
    coro_t::call_type coro1( scheduler);
    c1 = & coro1;
    coro1();

    std::cout << "Done" << std::endl;

    return EXIT_SUCCESS;
}