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
typedef boost::coroutines::symmetric_coroutine< void >  coro_t;

/* Test Params */
int size = 4096;
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

/* Event Preempt Signal Test */
int cur_request = 0;
dml::handler<dml::mem_move_operation, std::allocator<std::uint8_t>> handlers[num_requests]; /* I want to have multiple requests in flight and have requests being preempted */
int next_response_idx = 0;


void request_fn( coro_t::yield_type &yield){
  int next_submit_idx = cur_request;

  auto src_data = std::vector<uint8_t>('a',size);
  auto dst_data = std::vector<uint8_t>(src_data.size());

  #ifdef BREAKDOWN
  before_submit[cur_sample] = __rdtsc();
  #endif

  handlers[next_submit_idx] = dml::submit<dml::hardware>(dml::mem_move,
                                        dml::make_view(src_data),
                                        dml::make_view(dst_data));

  cur_request++;

  #ifdef BREAKDOWN
  before_yield[cur_sample] = __rdtsc();
  #endif
  yield();
  #ifdef BREAKDOWN
  after_resume[cur_sample] = __rdtsc();
  #endif




}

void scheduler( coro_t::yield_type & yield)
{
  coro_t::call_type request_coro{request_fn};

  #ifdef BREAKDOWN
  request_start_time[cur_sample] = __rdtsc();
  #endif
  request_coro();
  #ifdef BREAKDOWN
  after_yield[cur_sample] = __rdtsc();
  #endif

  /*
  (Hierarchical CR Preemption Signaling)
  The preemption signal could be a completion record itself - we may not actually be waiting on a preemption signal from the dispatcher
  The reason we may not is if a hierarchical scheduling policy emulating c-FCFS tried to enable the worker to simply preempt and reschedule the yielding
  request without explicit dispatcher signalling

  We assume a hierarchical accelerator-aware scheduling policy where the preemption signal is the CR arrival itself.
  No preemption signal from the centralized dispatcher is needed to swap the context.
  */

  auto result = handlers[next_response_idx].get();
  if(result.status != dml::status_code::ok){
    std::cerr<<"DSA Offload Failed\n";
    exit(-1);
  }
  /* By blocking when checking for responses in the scheduler, we may inadvertently waste CPU time by not scheduling a ready request */
  /* We need an async check API that allows ___ */
  next_response_idx++;


  #ifdef BREAKDOWN
  before_resume[cur_sample] = __rdtsc();
  #endif
  request_coro();
  #ifdef BREAKDOWN
  after_complete[cur_sample] = __rdtsc();
  #endif

}


int main( int argc, char * argv[])
{
  int size = 1024;
  int core = 5;



  for(int i=0; i<num_samples; i++){
    coro_t::call_type coro1(scheduler);
    coro1();
    cur_sample++;
  }

  #ifdef BREAKDOWN
  uint64_t avg = 0;
  uint64_t yield_to_submit[num_samples];

  avg_samples_from_arrays(yield_to_submit, avg, before_yield, before_submit,num_samples);
  std::cout << "SubmitCycles: " << avg << std::endl;
  avg_samples_from_arrays(yield_to_submit, avg, after_yield, before_yield,num_samples);
  std::cout << "ContextSwitchToWorkerSchedulerCycles: " << avg << std::endl;
  avg_samples_from_arrays(yield_to_submit, avg,before_resume,  after_yield ,num_samples);
  std::cout << "WorkerWaitCycles: " << avg << std::endl;
  avg_samples_from_arrays(yield_to_submit, avg, after_resume, before_resume,num_samples);
  std::cout << "ContextSwitchToRequestCycles: " << avg << std::endl;

  std::cout << std::endl;
  avg_samples_from_arrays(yield_to_submit, avg, before_resume, before_yield,num_samples);
  std::cout << "ActualOffloadCycles: " << avg << std::endl;

  #endif

    return EXIT_SUCCESS;
}