#include <boost/coroutine/all.hpp>

#include <cstdlib>
#include <iostream>
#include <immintrin.h>
#include <vector>

#include <x86intrin.h>
#include <thread>
#include <pthread.h>
#include <sched.h>

#include <zlib.h>


/* Test Params */
static constexpr int num_requests = 1000;
#define BUFFER_SIZE  4096 // 2 MB

uint8_t * source;
uint8_t * destination;


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
uint64_t before_cpu[num_samples];
uint64_t after_cpu[num_samples];
#endif

int main( int argc, char * argv[])
{
  int core = 5;
  /* perform memcpy */

  for(int i=0; i<num_requests; i++){
    source = (uint8_t *)malloc(BUFFER_SIZE);;
    destination = (uint8_t *)malloc(BUFFER_SIZE);;
    for(int i = 0; i < BUFFER_SIZE; i++){
        source[i] = i % 256;
    }

    #ifdef BREAKDOWN
    before_cpu[cur_sample] = __rdtsc();
    #endif

    memcpy(destination, source, BUFFER_SIZE);
    uint64_t crc;
    crc = crc32(0L, Z_NULL, 0);
    crc = crc32(crc, destination, BUFFER_SIZE);

    #ifdef BREAKDOWN
    after_cpu[cur_sample] = __rdtsc();
    cur_sample++;
    #endif

  }

  uint64_t avg;
  uint64_t yield_to_submit[num_requests];
  avg_samples_from_arrays(yield_to_submit, avg, after_cpu, before_cpu,num_samples);
  std::cout<< "CPU Compress and CRC Cycles: " << avg << std::endl;


  return EXIT_SUCCESS;
}