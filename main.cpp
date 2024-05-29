#include <boost/coroutine/all.hpp>

#include <cstdlib>
#include <iostream>
#include <immintrin.h>
#include <vector>

#include <x86intrin.h>
#include <thread>
#include <pthread.h>
#include <sched.h>

#include <boost/bind.hpp>

#include "dml/dml.hpp"
#include "dml/dml.h"
typedef boost::coroutines::symmetric_coroutine< void >  coro_t;

/* Coroutine for Request 2 */

/* Test Params */
int size = 2048 * 1024;
static constexpr int num_requests = 1024;

/* Wait Functions */
static __always_inline void umonitor(const volatile void *addr)
{
	asm volatile(".byte 0xf3, 0x48, 0x0f, 0xae, 0xf0" : : "a"(addr));
}

static __always_inline int umwait(unsigned long timeout, unsigned int state)
{
	uint8_t r;
	uint32_t timeout_low = (uint32_t)timeout;
	uint32_t timeout_high = (uint32_t)(timeout >> 32);

	asm volatile(".byte 0xf2, 0x48, 0x0f, 0xae, 0xf1\t\n"
		"setc %0\t\n"
		: "=r"(r)
		: "c"(state), "a"(timeout_low), "d"(timeout_high));
	return r;
}


/* Stats Utils */
template <typename NUMTYPE>
static inline void gen_diff_array(NUMTYPE dst_array, NUMTYPE array1, NUMTYPE array2, int size)
{
  for(int i=0; i<size; i++){ dst_array[i] = array2[i] - array1[i]; }
}

#define do_sum_array(accum,array,iter) accum = 0; \
 for (int i=1; i<iter; i++){ accum+=array[i]; } \
 accum /= iter
#define do_avg(sum, itr) (sum/itr)

#define avg_samples_from_arrays(yield_to_submit, avg, before_yield, before_submit, num_samples) \
  gen_diff_array(yield_to_submit, before_submit, before_yield, num_samples); \
  do_sum_array(avg, yield_to_submit, num_samples); \
  do_avg(avg, num_samples);

#ifdef BREAKDOWN
static constexpr int num_samples = num_requests;
int cur_sample = 0;
uint64_t before_submit[num_samples];
uint64_t before_yield[num_samples];
uint64_t after_yield[num_samples];
uint64_t after_resume[num_samples];
uint64_t request_start_time[num_samples];
uint64_t before_resume[num_samples];
uint64_t after_complete[num_samples];
#endif


int cur_request = 0;
dml::handler<dml::mem_copy_operation, std::allocator<std::uint8_t>> handlers[num_requests]; /* I want to have multiple requests in flight and have requests being preempted */
int next_response_idx = 0;

coro_t::call_type * c2 = 0;
coro_t::call_type *c1 = 0;


void request_2_fn( coro_t::yield_type &yield){


  #ifdef BREAKDOWN
  after_yield[cur_sample] = __rdtsc();
  #endif
  auto result = handlers[next_response_idx].get();
  if(result.status != dml::status_code::ok){
    std::cerr<<"DSA Offload Failed\n";
    exit(-1);
  }

  next_response_idx++;

  #ifdef BREAKDOWN
  before_resume[cur_sample] = __rdtsc();
  #endif

  yield( *c1);
}

void request_fn( coro_t::yield_type &yield){

  #ifdef BREAKDOWN
  request_start_time[cur_sample] = __rdtsc();
  #endif

  int next_submit_idx = cur_request;

  auto src_data = std::vector<uint8_t>('a',size);
  auto dst_data = std::vector<uint8_t>(src_data.size());

  #ifdef BREAKDOWN
  before_submit[cur_sample] = __rdtsc();
  #endif

  handlers[next_submit_idx] = dml::submit<dml::hardware>(dml::mem_copy,
                                        dml::make_view(src_data),
                                        dml::make_view(dst_data));

  cur_request++;

  #ifdef BREAKDOWN
  before_yield[cur_sample] = __rdtsc();
  #endif
  yield(*c2);
  #ifdef BREAKDOWN
  after_resume[cur_sample] = __rdtsc();
  #endif
}


int main( int argc, char * argv[])
{
  int core = 5;



  for(int i=0; i<num_samples; i++){
    coro_t::call_type coro2(request_2_fn);
    coro_t::call_type coro1(request_fn);
    c2 = &coro2;
    c1 = &coro1;
    coro1();
    cur_sample++;
  }

  #ifdef BREAKDOWN
  uint64_t avg = 0;
  uint64_t yield_to_submit[num_samples];

  avg_samples_from_arrays(yield_to_submit, avg, before_yield, request_start_time,num_samples);
  std::cout<< "PrepCycles: " << avg << std::endl;
  avg_samples_from_arrays(yield_to_submit, avg, before_yield, before_submit,num_samples);
  std::cout << "SubmitCycles: " << avg << std::endl;
  avg_samples_from_arrays(yield_to_submit, avg, after_yield, before_yield,num_samples);
  std::cout << "ContextSwitchToRequest1: " << avg << std::endl;
  avg_samples_from_arrays(yield_to_submit, avg,before_resume,  after_yield ,num_samples);
  std::cout << "Request1Cycles: " << avg << std::endl;
  avg_samples_from_arrays(yield_to_submit, avg, after_resume, before_resume,num_samples);
  std::cout << "ContextSwitchBackToRequest0: " << avg << std::endl;

  avg_samples_from_arrays(yield_to_submit, avg, before_resume, before_yield,num_samples);
  std::cout<< "ActualOffloadCycles: " << avg << std::endl;
  #endif

    return EXIT_SUCCESS;
}