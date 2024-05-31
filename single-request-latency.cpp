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

/* Test Params */
int size = 4096;
static constexpr int num_requests = 1000;

/* C API Batch Test Params */
#define BUFFER_SIZE  4096
#define PADDING_SIZE 4096 // DML_OP_DUALCAST requirement "dst1 and dst2 address bits 11:0 must be the same"
#define BATCH_COUNT  5u   // 7 ops for this batch operation
#define PATTERN_SIZE 8u   // pattern size is always 8

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

uint64_t scheduler_start_time[num_samples];
uint64_t request_start_time[num_samples];
uint64_t scheduler_resume_time[num_samples];
uint64_t scheduler_resume_2_time[num_samples];
uint64_t scheduler_end_time[num_samples];

uint64_t before_resume[num_samples];
uint64_t after_complete[num_samples];
#endif

#ifdef BREAKDOWN
uint64_t before_job_alloc[num_samples];
uint64_t after_job_alloc[num_samples];

uint64_t before_job_prepare[num_samples];
uint64_t after_job_prepare[num_samples];

uint64_t before_job_submit[num_samples];
uint64_t after_job_submit[num_samples];

uint64_t before_job_wait[num_samples];
uint64_t after_job_wait[num_samples];
#endif

int main( int argc, char * argv[])
{
  int core = 5;

  dml_job_t *dml_job_ptr;
  dml_status_t status;
  uint32_t job_size_ptr;


  uint8_t source      [BUFFER_SIZE];
  uint8_t destination [BUFFER_SIZE];

  for(int i = 0; i < BUFFER_SIZE; i++){
      source[i] = i % 256;
  }

  for(int i=0; i<num_samples; i++){
    uint32_t crc = 1;

    cur_sample = i;

    #ifdef BREAKDOWN
    before_job_alloc[cur_sample] = __rdtsc();
    #endif
    status = dml_get_job_size(DML_PATH_HW, &job_size_ptr);
    if(status != DML_STATUS_OK){
      std::cerr << "Job size determination failed\n";
    }
    dml_job_ptr = (dml_job_t *) malloc(job_size_ptr);

    #ifdef BREAKDOWN
    after_job_alloc[cur_sample] = __rdtsc();
    #endif

    #ifdef BREAKDOWN
    before_job_prepare[cur_sample] = __rdtsc();
    #endif

    status = dml_init_job(DML_PATH_HW, dml_job_ptr);
    if(status != DML_STATUS_OK){
      std::cerr << "Job initialization failed\n";
    }
    dml_job_ptr->operation = DML_OP_COPY_CRC;
    dml_job_ptr->source_first_ptr       = source;
    dml_job_ptr->destination_first_ptr  = destination;
    dml_job_ptr->source_length          = BUFFER_SIZE;
    dml_job_ptr->crc_checksum_ptr       = &crc;

    #ifdef BREAKDOWN
    after_job_prepare[cur_sample] = __rdtsc();
    #endif


    status = dml_submit_job(dml_job_ptr);
    if (DML_STATUS_OK != status) {
        printf("An error (%u) occured during job submission .\n", status);
        dml_finalize_job(dml_job_ptr);
        free(dml_job_ptr);
        return 1;
    }
    #ifdef BREAKDOWN
    after_job_submit[cur_sample] = __rdtsc();
    #endif



    #ifdef BREAKDOWN
    before_job_wait[cur_sample] = __rdtsc();
    #endif
    status  = dml_wait_job(dml_job_ptr, DML_WAIT_MODE_BUSY_POLL);

    if(status != DML_STATUS_OK){
      std::cerr<<"DSA Offload Failed\n";
      exit(-1);
    }

    #ifdef BREAKDOWN
    after_job_wait[cur_sample] = __rdtsc();
    #endif



    for(int i = 0; i < BUFFER_SIZE; i++){
        if(destination[i] != source[i]){
            printf("Error: Operation result is incorrect.\n");
            dml_finalize_job(dml_job_ptr);
            free(dml_job_ptr);
            return 1;
        }
    }
    status = dml_finalize_job(dml_job_ptr);
    if (DML_STATUS_OK != status) {
        printf("An error (%u) occured during job finalization.\n", status);
        free(dml_job_ptr);
        return 1;
    }
    free(dml_job_ptr);
  }


  #ifdef BREAKDOWN
  uint64_t avg = 0;
  uint64_t yield_to_submit[num_samples];

  avg_samples_from_arrays(yield_to_submit, avg, after_job_alloc, before_job_alloc,num_samples);
  std::cout<< "JobAllocCycles: " << avg << std::endl;
  avg_samples_from_arrays(yield_to_submit, avg, after_job_prepare, before_job_prepare,num_samples);
  std::cout<< "JobPrepCycles: " << avg << std::endl;
  avg_samples_from_arrays(yield_to_submit, avg, after_job_submit, after_job_prepare,num_samples);
  std::cout << "SubmitCycles: " << avg << std::endl;
  avg_samples_from_arrays(yield_to_submit, avg, after_job_wait, before_job_wait,num_samples);
  std::cout<< "WaitCycles: " << avg << std::endl;
  #endif

    return EXIT_SUCCESS;
}