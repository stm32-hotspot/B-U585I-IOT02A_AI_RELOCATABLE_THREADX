/**
 ******************************************************************************
 * @file    aiSystemPerformance_RELOC.c
 * @author  MCD/AIS Team
 * @brief   AI System perf. application (entry points) - Relocatable network
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019,2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software is licensed under terms that can be found in the LICENSE file in
 * the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/*
 * Description
 *
 * - Entry points) for the AI System Perf. of a relocatable network object.
 *   Support for a simple relocatable network (no multiple network support).
 *
 * - Allows to report the inference time (global and by layer/operator)
 *   with random inputs (outputs are skipped). Heap and stack usage is also
 *   reported.
 *
 * History:
 *  - v1.0 - Initial version (based on the original aiSystemPerformance v5.1)
 *  - v1.1 - Use the fix cycle count overflow support for time per layer
 *  - v1.2 - Add support to use SYSTICK only (remove direct call to DWT fcts)
 *  - v2.0 - Update to support fragmented activations/weights buffer
 *           activations and io buffers are fully handled by app_x-cube-ai.c/h files
 *           Add support to register the user call-backs to manage the CRC IP.
 *           Align code with the new ai_buffer struct definition
 */

/* System headers */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#if !defined(USE_OBSERVER)
#define USE_OBSERVER         1 /* 0: remove the registration of the user CB to evaluate the inference time by layer */
#endif
#define USE_CORE_CLOCK_ONLY  0 /* 1: remove usage of the HAL_GetTick() to evaluate the number of CPU clock. Only the Core
                                *    DWT IP is used. HAL_Tick() is requested to avoid an overflow with the DWT clock counter
                                *    (32b register) - USE_SYSTICK_ONLY should be set to 0.
                                */
#define USE_SYSTICK_ONLY     0 /* 1: use only the SysTick to evaluate the time-stamps (for Cortex-m0 based device, this define is forced) */

#define USE_CRC_USER_CB      1 /* 1: enable the registration of user cb to manage the accesses to the CRC IP */

#define USER_REL_COPY_MODE   0

#if !defined(APP_DEBUG)
#define APP_DEBUG     	     0 /* 1: add debug trace - application level */
#endif


#define _APP_ITER_     16  /* number of iteration for perf. test */

/* APP header files */
#include <aiSystemPerformance.h>
#include <aiTestUtility.h>
#include <aiTestHelper.h>

/* AI x-cube-ai files */
#include <app_x-cube-ai.h>

#include <ai_reloc_network.h>

/* Include the image of the relocatable network */
#include <network_img_rel.h>

/* -----------------------------------------------------------------------------
 * TEST-related definitions
 * -----------------------------------------------------------------------------
 */

#define _APP_VERSION_MAJOR_     (0x02)
#define _APP_VERSION_MINOR_     (0x00)
#define _APP_VERSION_   ((_APP_VERSION_MAJOR_ << 8) | _APP_VERSION_MINOR_)

#define _APP_NAME_     "AI system performance measurement (RELOC)"


#if AI_MNETWORK_NUMBER > 1 && !defined(AI_NETWORK_MODEL_NAME)
#error Only ONE network is supported (default c-name)
#endif

/* Global variables */
static bool observer_mode = true;
static bool profiling_mode = false;
static int  profiling_factor = 5;

#define STATIC_INPUT_DATA
#ifdef  STATIC_INPUT_DATA
 /* 5*25+3=128  */
const static float static_in_data_float[] = 
        {0.9660673 , 0.16911239, 0.29774541, 0.17224806, 0.74087739,
        0.8355425 , 0.12831554, 0.71952322, 0.32432654, 0.04200903,
        0.03119804, 0.59721436, 0.01782878, 0.23465973, 0.74782444,
        0.48006467, 0.33889812, 0.51989522, 0.7471727 , 0.2103228 ,
        0.48724606, 0.09139513, 0.41956972, 0.28943722, 0.3605218 ,
        0.67587533, 0.84642762, 0.85823529, 0.22001298, 0.71706064,
        0.94601753, 0.19077173, 0.05605285, 0.8805222 , 0.11730031,
        0.90200733, 0.27306643, 0.67840648, 0.4162532 , 0.40381544,
        0.16514668, 0.57341431, 0.38224073, 0.47617894, 0.26218297,
        0.49816117, 0.52441995, 0.06618827, 0.67733671, 0.17172281,
        0.01176868, 0.40305017, 0.82064255, 0.82858677, 0.04727406,
        0.99411882, 0.12744688, 0.95038618, 0.08928988, 0.57609561,
        0.43212584, 0.79557377, 0.15436037, 0.22025408, 0.25635702,
        0.28297777, 0.25678928, 0.63142759, 0.51440852, 0.05080481,
        0.33761543, 0.45989589, 0.45711661, 0.57288413, 0.60313534,
        0.4673632 , 0.0651958 , 0.02914347, 0.48756748, 0.5947893 ,
        0.5325755 , 0.327089  , 0.74533665, 0.90384461, 0.56344434,
        0.66781318, 0.04880192, 0.97360023, 0.98702051, 0.17730701,
        0.69719279, 0.97143772, 0.8403528 , 0.9578903 , 0.31548462,
        0.74951748, 0.8720388 , 0.46318353, 0.62883651, 0.34830131,
        0.57014835, 0.03838273, 0.41251491, 0.25915901, 0.11056856,
        0.45111699, 0.26260828, 0.05413052, 0.58366193, 0.29080595,
        0.81272205, 0.94622643, 0.27694987, 0.16326789, 0.42130382,
        0.51568054, 0.60574114, 0.104586  , 0.23794517, 0.03864999,
        0.00548758, 0.54016795, 0.93642904, 0.14118457, 0.97582098,
        0.95990026, 0.00179961, 0.5434109};
#endif

#define STATIC_OUTPUT_DATA
#ifdef  STATIC_OUTPUT_DATA

#include <network.h> 

const static float reference_out_data[] = {0.06708992, 0.9329101};  //reference out val for static_in_data_float
AI_ALIGNED(4) static float out_data[AI_NETWORK_OUT_1_SIZE];  

#endif

/* -----------------------------------------------------------------------------
 * Object definition/declaration for AI-related execution context
 * -----------------------------------------------------------------------------
 */

#if defined(USE_OBSERVER) && USE_OBSERVER == 1

struct u_node_stat {
  uint64_t dur;
  uint32_t n_runs;
};

struct u_observer_ctx {
  uint64_t n_cb;
  uint64_t start_t;
  uint64_t u_dur_t;
  uint64_t k_dur_t;
  struct u_node_stat *nodes;
};

static struct u_observer_ctx u_observer_ctx;

#endif /* USE_OBSERVER */

struct ai_network_exec_ctx {
  ai_handle handle;
  ai_network_report report;
} net_exec_ctx[1] = {0};

#if 0
/* RT Network buffer to relocatable network instance */
#if defined(USER_REL_COPY_MODE) && USER_REL_COPY_MODE == 1
AI_ALIGNED(32)
uint8_t reloc_ram[AI_NETWORK_RELOC_RAM_SIZE_COPY];
#else
AI_ALIGNED(32)
uint8_t reloc_ram[AI_NETWORK_RELOC_RAM_SIZE_XIP];
#endif
#endif

/* -----------------------------------------------------------------------------
 * Observer-related functions
 * -----------------------------------------------------------------------------
 */
#if defined(USE_OBSERVER) && USE_OBSERVER == 1

/* User callback for observer */
static ai_u32 user_observer_cb(const ai_handle cookie,
    const ai_u32 flags,
    const ai_observer_node *node) {

  struct u_observer_ctx *u_obs;

  volatile uint64_t ts = cyclesCounterEnd(); /* time stamp entry */

  u_obs = (struct u_observer_ctx *)cookie;
  u_obs->n_cb += 1;

  if (flags & AI_OBSERVER_POST_EVT) {
    const uint64_t end_t = ts - u_obs->start_t;
    u_obs->k_dur_t += end_t;
    u_obs->nodes[node->c_idx].dur += end_t;
    u_obs->nodes[node->c_idx].n_runs += 1;
  }

  u_obs->start_t = cyclesCounterEnd();    /* time stamp exit */
  u_obs->u_dur_t += u_obs->start_t  - ts; /* accumulate cycles used by the CB */
  return 0;
}


void aiObserverInit(struct ai_network_exec_ctx *ctx)
{
  ai_bool res;
  int sz;

  if ((ctx->handle == AI_HANDLE_NULL) || !ctx->report.n_nodes)
    return;

  memset((void *)&u_observer_ctx, 0, sizeof(struct u_observer_ctx));

  /* allocate resources to store the state of the nodes */
  sz = ctx->report.n_nodes * sizeof(struct u_node_stat);
  u_observer_ctx.nodes = (struct u_node_stat*)malloc(sz);
  if (!u_observer_ctx.nodes) {
    LC_PRINT("W: enable to allocate the u_node_stats (sz=%d) ..\r\n", sz);
    return;
  }

  memset(u_observer_ctx.nodes, 0, sz);

  /* register the callback */
  res = ai_rel_platform_observer_register(ctx->handle, user_observer_cb,
      (ai_handle)&u_observer_ctx, AI_OBSERVER_PRE_EVT | AI_OBSERVER_POST_EVT);
  if (!res) {
    LC_PRINT("W: unable to register the user CB\r\n");
    free(u_observer_ctx.nodes);
    u_observer_ctx.nodes = NULL;
    return;
  }
}

extern const char* ai_layer_type_name(const int type);

void aiObserverDone(struct ai_network_exec_ctx *ctx)
{
  struct dwtTime t;
  uint64_t cumul;
  ai_observer_node node_info;

  if ((ctx->handle == AI_HANDLE_NULL) || !ctx->report.n_nodes || !u_observer_ctx.nodes)
    return;

  ai_rel_platform_observer_unregister(ctx->handle, user_observer_cb,
      (ai_handle)&u_observer_ctx);


  LC_PRINT("\r\n Inference time by c-node\r\n");
  dwtCyclesToTime(u_observer_ctx.k_dur_t / u_observer_ctx.nodes[0].n_runs, &t);
  LC_PRINT("  kernel  : %d.%03dms (time passed in the c-kernel fcts)\r\n", t.s * 1000 + t.ms, t.us);
  dwtCyclesToTime(u_observer_ctx.u_dur_t / u_observer_ctx.nodes[0].n_runs, &t);
  LC_PRINT("  user    : %d.%03dms (time passed in the user cb)\r\n", t.s * 1000 + t.ms, t.us);
#if APP_DEBUG == 1
  LC_PRINT("  cb #    : %d\n", (int)u_observer_ctx.n_cb);
#endif

  LC_PRINT("\r\n %-6s%-20s%-7s  %s\r\n", "c_id", "type", "id", "time (ms)");
  LC_PRINT(" ---------------------------------------------------\r\n");

  cumul = 0;
  node_info.c_idx = 0;
  while (ai_rel_platform_observer_node_info(ctx->handle, &node_info)) {

    struct u_node_stat *sn = &u_observer_ctx.nodes[node_info.c_idx];
    const char *fmt;
    cumul +=  sn->dur;
    dwtCyclesToTime(sn->dur / (uint64_t)sn->n_runs, &t);
    if ((node_info.type & (ai_u16)0x8000) >> 15)
      fmt = " %-6dTD-%-17s%-5d %6d.%03d %6.02f %c\r\n";
    else
      fmt = " %-6d%-20s%-5d %6d.%03d %6.02f %c\r\n";
#ifdef NO_CLEANUP_DEPENDENCY
    LC_PRINT(fmt, node_info.c_idx,
        ai_layer_type_name(node_info.type  & (ai_u16)0x7FFF),
        (int)node_info.id,
        t.s * 1000 + t.ms, t.us,
        ((float)u_observer_ctx.nodes[node_info.c_idx].dur * 100.0f) / (float)u_observer_ctx.k_dur_t,
        '%');
#endif
    node_info.c_idx++;
  }

  LC_PRINT(" -------------------------------------------------\r\n");
  cumul /= u_observer_ctx.nodes[0].n_runs;
  dwtCyclesToTime(cumul, &t);
  LC_PRINT(" %31s %6d.%03d ms\r\n", "", t.s * 1000 + t.ms, t.us);

  free(u_observer_ctx.nodes);
  memset((void *)&u_observer_ctx, 0, sizeof(struct u_observer_ctx));

  return;
}

#endif /* USE_OBSERVER */

#if defined(USE_CRC_USER_CB) && USE_CRC_USER_CB == 1
uint32_t user_acquire_crc(void)
{
  /* default behavior - CRC IP is always ON */
  return 0x1;
}

void user_release_crc(uint32_t id)
{
}
#endif

static int aiBootstrap(struct ai_network_exec_ctx *ctx)
{
  extern TX_BYTE_POOL            AIBytePool;  
  ai_error err;
  ai_handle weights_addr;
  ai_rel_network_info rt_info;
  void * mem;
  
  /* Creating an instance of the network ------------------------- */
  LC_PRINT("\r\nInstancing the network (reloc)..\r\n");

  err = ai_rel_network_rt_get_info(ai_network_reloc_img_get(), &rt_info);
  if (err.type != AI_ERROR_NONE) {
      aiLogErr(err, "ai_rel_network_rt_get_info");
      return -1;
    }
  
#if defined(USER_REL_COPY_MODE) && USER_REL_COPY_MODE == 1  
  /* allocate 32 bytes aligned RAM mem for AI from the AI byte pool */
  if (tx_byte_allocate(&AIBytePool, (VOID **) &mem, rt_info.rt_ram_copy + 31, TX_NO_WAIT) != TX_SUCCESS)
  {
    Error_Handler();
  }    
  void * reloc_ram = (void *)(((uintptr_t)mem+31) & ~ (uintptr_t)0x0F);  
  
#if defined(USE_CRC_USER_CB) && USE_CRC_USER_CB == 1
  err = ai_rel_network_load_and_create_ext(ai_network_reloc_img_get(),
      reloc_ram, rt_info.rt_ram_copy/*AI_NETWORK_RELOC_RAM_SIZE_COPY*/, AI_RELOC_RT_LOAD_MODE_COPY,
      &user_acquire_crc, &user_release_crc, &ctx->handle);
#else
  err = ai_rel_network_load_and_create(ai_network_reloc_img_get(),
      reloc_ram, rt_info.rt_ram_copy/*AI_NETWORK_RELOC_RAM_SIZE_COPY*/, AI_RELOC_RT_LOAD_MODE_COPY,
      &ctx->handle);
#endif
#else   /* AI in XIP mode */
  /* allocate 32 bytes aligned RAM mem for AI from the AI byte pool */
  if (tx_byte_allocate(&AIBytePool, (VOID **) &mem, rt_info.rt_ram_xip + 31, TX_NO_WAIT) != TX_SUCCESS)
  {
    Error_Handler();
  }    
  void * reloc_ram = (void *)(((uintptr_t)mem+31) & ~ (uintptr_t)0x0F);
  
#if defined(USE_CRC_USER_CB) && USE_CRC_USER_CB == 1
  err = ai_rel_network_load_and_create_ext(ai_network_reloc_img_get(),
      reloc_ram, rt_info.rt_ram_xip/*AI_NETWORK_RELOC_RAM_SIZE_XIP*/, AI_RELOC_RT_LOAD_MODE_XIP,
      &user_acquire_crc, &user_release_crc, &ctx->handle);
#else
  err = ai_rel_network_load_and_create(ai_network_reloc_img_get(),
      reloc_ram, rt_info.rt_ram_xip/*AI_NETWORK_RELOC_RAM_SIZE_XIP*/, AI_RELOC_RT_LOAD_MODE_XIP,
      &ctx->handle);
#endif
#endif
  if (err.type != AI_ERROR_NONE) {
    aiLogErr(err, "ai_rel_network_load_and_create");
    return -1;
  }

  /* test returned err value (debug purpose) */
  err = ai_rel_network_get_error(ctx->handle);
  if (err.type != AI_ERROR_NONE) {
    aiLogErr(err, "ai_rel_network_get_error");
    return -1;
  }

  /* Initialize the instance --------------------------------------- */
  LC_PRINT("Initializing the network\r\n");
  /* in case of single Netw+Weights reloc file */
  weights_addr = rt_info.weights;
#if defined(AI_NETWORK_DATA_WEIGHTS_GET_FUNC)
  /* in case of separate Netw and Weights reloc files (--binary option) */
  /* the real weights address in flash is returned by a custom funct (after having flashed weights file) */
  if (!weights_addr)
    weights_addr = (ai_handle)AI_NETWORK_DATA_WEIGHTS_GET_FUNC();
#endif

  if (!ai_rel_network_init(ctx->handle,
      &weights_addr, data_activations0)) {
    err = ai_rel_network_get_error(ctx->handle);
    aiLogErr(err, "ai_rel_network_init");
    ai_rel_network_destroy(ctx->handle);
    ctx->handle = AI_HANDLE_NULL;
    return -2;
  }

  /* Display the network info -------------------------------------- */
  if (ai_rel_network_get_report(ctx->handle, &ctx->report)) {
    aiPrintNetworkInfo(&ctx->report);
  } else {
    err = ai_rel_network_get_error(ctx->handle);
    aiLogErr(err, "ai_rel_network_get_info");
    ai_rel_network_destroy(ctx->handle);
    ctx->handle = AI_HANDLE_NULL;
    return -3;
  }

  return 0;
}

static void aiDone(struct ai_network_exec_ctx *ctx)
{
  ai_error err;

  /* Releasing the instance(s) ------------------------------------- */
  LC_PRINT("Releasing the instance...\r\n");

  if (ctx->handle != AI_HANDLE_NULL) {
    if (ai_rel_network_destroy(ctx->handle)
        != AI_HANDLE_NULL) {
      err = ai_rel_network_get_error(ctx->handle);
      aiLogErr(err, "ai_rel_network_destroy");
    }
    ctx->handle = AI_HANDLE_NULL;
  }
}

static int aiInit(void)
{
  int res;

  aiPlatformVersion();

  net_exec_ctx[0].handle = AI_HANDLE_NULL;
  res = aiBootstrap(&net_exec_ctx[0]);

  return res;
}

static void aiDeInit(void)
{
  aiDone(&net_exec_ctx[0]);
}


/* -----------------------------------------------------------------------------
 * Specific APP/test functions
 * -----------------------------------------------------------------------------
 */
static int aiTestPerformance(void)
{
  int iter;
  ai_i32 batch;
  int niter;

  struct dwtTime t;
  uint64_t tcumul;
  uint64_t tend;
  uint32_t cmacc;
  uint32_t error = 0;

  struct ai_network_exec_ctx *ctx = &net_exec_ctx[0];

#if defined(USE_OBSERVER) && USE_OBSERVER == 1
  int observer_heap_sz = 0UL;
#endif

  ai_buffer ai_input[AI_NETWORK_IN_NUM];
  ai_buffer ai_output[AI_NETWORK_OUT_NUM];

  if (ctx->handle == AI_HANDLE_NULL) {
    LC_PRINT("E: network handle is NULL\r\n");
    return -1;
  }

  MON_STACK_INIT();

  if (profiling_mode)
    niter = _APP_ITER_ * profiling_factor;
  else
    niter = _APP_ITER_;

  LC_PRINT("\r\nRunning PerfTest on \"%s\" with random inputs (%d iterations)...\r\n",
      ctx->report.model_name, niter);


#if APP_DEBUG == 1
  MON_STACK_STATE("stack before test");
#endif


  MON_STACK_CHECK0();

  /* reset/init cpu clock counters */
  tcumul = 0ULL;

  MON_STACK_MARK();

  /* Fill the input tensor descriptors */
  for (int i = 0; i < ctx->report.n_inputs; i++) {
    ai_input[i] = ctx->report.inputs[i];
    if (ctx->report.inputs[i].data)
      ai_input[i].data = AI_HANDLE_PTR(ctx->report.inputs[i].data);
    else
      ai_input[i].data = AI_HANDLE_PTR(data_ins[i]);
  }

#ifdef STATIC_OUTPUT_DATA  
  
  /* Fill the output tensor descriptors */
  for (int i = 0; i < ctx->report.n_outputs; i++) {
    ai_output[i] = ctx->report.outputs[i];
    if (ctx->report.outputs[i].data)
//      ai_output[i].data = AI_HANDLE_PTR(ctx->report.outputs[i].data);
      ai_output[i].data = AI_HANDLE_PTR(out_data);      
    else
      ai_output[i].data = AI_HANDLE_PTR(data_outs[i]);
  }

  if (profiling_mode) {
    LC_PRINT("Profiling mode (%d)...\r\n", profiling_factor);
    fflush(stdout);
  }

#else

  /* Fill the output tensor descriptors */
  for (int i = 0; i < ctx->report.n_outputs; i++) {
    ai_output[i] = ctx->report.outputs[i];
    if (ctx->report.outputs[i].data)
      ai_output[i].data = AI_HANDLE_PTR(ctx->report.outputs[i].data);
    else
      ai_output[i].data = AI_HANDLE_PTR(data_outs[i]);
  }

  if (profiling_mode) {
    LC_PRINT("Profiling mode (%d)...\r\n", profiling_factor);
    fflush(stdout);
  }
 
#endif  
  
#if defined(USE_OBSERVER) && USE_OBSERVER == 1
  /* Enable observer */
  if (observer_mode) {
    MON_ALLOC_RESET();
    MON_ALLOC_ENABLE();
    aiObserverInit(ctx);
    observer_heap_sz = MON_ALLOC_MAX_USED();
  }
#endif /* USE_OBSERVER */

  MON_ALLOC_RESET();

  /* Main inference loop */
  for (iter = 0; iter < niter; iter++) {

#ifdef STATIC_INPUT_DATA
    
    /* Fill input tensors with fixed pattern data */
    for (int i = 0; i < ctx->report.n_inputs; i++) {
      const ai_buffer_format fmt = AI_BUFFER_FORMAT(&ai_input[i]);
      ai_i8 *in_data = (ai_i8 *)ai_input[i].data;
      ai_size j = 0;
#ifdef NO_CLEANUP_DEPENDENCY
      for (; j < AI_BUFFER_SIZE(&ai_input[i]); ++j) {
#else
      for (; j < 128; ++j) {      
#endif

        /* uniform distribution between -1.0 and 1.0 */
//        const float v = 2.0f * (ai_float) rand() / (ai_float) RAND_MAX - 1.0f;
          const float v = static_in_data_float[j];
        if  (AI_BUFFER_FMT_GET_TYPE(fmt) == AI_BUFFER_FMT_TYPE_FLOAT) {
          *(ai_float *)(in_data + j * 4) = v;
        }
        else {
          in_data[j] = (ai_i8)(v * 127);
          if (AI_BUFFER_FMT_GET_TYPE(fmt) == AI_BUFFER_FMT_TYPE_BOOL) {
            in_data[j] = (in_data[j] > 0)?(ai_i8)1:(ai_i8)0;
          }
        }
      }
    }    
    
#else      
    /* Fill input tensors with random data */
    for (int i = 0; i < ctx->report.n_inputs; i++) {
      const ai_buffer_format fmt = AI_BUFFER_FORMAT(&ai_input[i]);
      ai_i8 *in_data = (ai_i8 *)ai_input[i].data;
      for (ai_size j = 0; j < AI_BUFFER_SIZE(&ai_input[i]); ++j) {
        /* uniform distribution between -1.0 and 1.0 */
        const float v = 2.0f * (ai_float) rand() / (ai_float) RAND_MAX - 1.0f;
        if  (AI_BUFFER_FMT_GET_TYPE(fmt) == AI_BUFFER_FMT_TYPE_FLOAT) {
          *(ai_float *)(in_data + j * 4) = v;
        }
        else {
          in_data[j] = (ai_i8)(v * 127);
          if (AI_BUFFER_FMT_GET_TYPE(fmt) == AI_BUFFER_FMT_TYPE_BOOL) {
            in_data[j] = (in_data[j] > 0)?(ai_i8)1:(ai_i8)0;
          }
        }
      }
    }
#endif
    MON_ALLOC_ENABLE();

    // free(malloc(20));

    cyclesCounterStart();
    batch = ai_rel_network_run(ctx->handle, ai_input, ai_output);
    if (batch != 1) {
      aiLogErr(ai_rel_network_get_error(ctx->handle),
          "ai_rel_network_run");
      break;
    }
    tend = cyclesCounterEnd();
    
    MON_ALLOC_DISABLE();

    tcumul += tend;

    dwtCyclesToTime(tend, &t);
    
#if defined (STATIC_OUTPUT_DATA) && defined  (STATIC_INPUT_DATA)

    /* check output data */
    for (int i=0; i<AI_NETWORK_OUT_1_SIZE; i++)
    {
       if ((roundf(10000 * out_data[i]) / 10000) != 
           (roundf(10000 * reference_out_data[i]) / 10000)) error++;    
    }
       
#endif    
    
#if APP_DEBUG == 1
    LC_PRINT(" #%02d %8d.%03dms (%ld cycles)\r\n", iter,
        t.ms, t.us, (long)tend);
#else
    if (!profiling_mode) {
      if (t.s > 10)
        niter = iter;
//      LC_PRINT(".");
      fflush(stdout);
    }
#endif
  } /* end of the main loop */

    /* print out data */
  printf (" %d run errors\n\r", error);
  for (int i=0; i<AI_NETWORK_OUT_1_SIZE; i++)
  {
    printf ("=> out_data[%d]: %.8f\n\r", i, (double)out_data[i]);    
  }
  
#if APP_DEBUG != 1
  LC_PRINT("\r\n");
#endif

  MON_STACK_EVALUATE();


  LC_PRINT("\r\n");

#if defined(USE_OBSERVER) && USE_OBSERVER == 1
  /* remove the user cb time */
  tcumul -= u_observer_ctx.u_dur_t;
#endif

  tcumul /= (uint64_t)iter;

  dwtCyclesToTime(tcumul, &t);

  LC_PRINT("Results for \"%s\" (R), %d inferences @%dMHz/%dMHz (complexity: %lu MACC)\r\n",
      ctx->report.model_name, (int)iter,
      (int)(HAL_RCC_GetSysClockFreq() / 1000000),
      (int)(HAL_RCC_GetHCLKFreq() / 1000000),
      (unsigned long)ctx->report.n_macc);

  LC_PRINT(" duration     : %d.%03d ms (average)\r\n", t.s * 1000 + t.ms, t.us);
  if (tcumul / 100000)
    LC_PRINT(" CPU cycles   : %ld%ld (average)\r\n",
      (unsigned long)(tcumul / 100000), (unsigned long)(tcumul - ((tcumul / 100000) * 100000)));
  else
    LC_PRINT(" CPU cycles   : %ld (average)\r\n", (unsigned long)(tcumul));
  LC_PRINT(" CPU Workload : %d%c (duty cycle = 1s)\r\n", (int)((tcumul * 100) / t.fcpu), '%');
  cmacc = (uint32_t)((tcumul * 100)/ ctx->report.n_macc);
  LC_PRINT(" cycles/MACC  : %d.%02d (average for all layers)\r\n",
      (int)(cmacc / 100), (int)(cmacc - ((cmacc / 100) * 100)));

  MON_STACK_REPORT();
  MON_ALLOC_REPORT();

#if defined(USE_OBSERVER) && USE_OBSERVER == 1
  LC_PRINT(" observer res : %d bytes used from the heap (%d c-nodes)\r\n", observer_heap_sz,
      (int)ctx->report.n_nodes);
  aiObserverDone(ctx);
#endif /* USE_OBSERVER */

  return 0;
}

/* -----------------------------------------------------------------------------
 * Basic interactive console
 * -----------------------------------------------------------------------------
 */

#define CONS_EVT_TIMEOUT    (0)
#define CONS_EVT_QUIT       (1)
#define CONS_EVT_RESTART    (2)
#define CONS_EVT_HELP       (3)
#define CONS_EVT_PAUSE      (4)
#define CONS_EVT_PROF       (5)
#define CONS_EVT_HIDE       (6)

#define CONS_EVT_UNDEFINED  (100)

#if 0
static int aiTestConsole(void)
{
  uint8_t c = 0;

  if (ioRawGetUint8(&c, 5000) == -1) /* Timeout */
    return CONS_EVT_TIMEOUT;

  if ((c == 'q') || (c == 'Q'))
    return CONS_EVT_QUIT;

  if ((c == 'd') || (c == 'D'))
    return CONS_EVT_HIDE;

  if ((c == 'r') || (c == 'R'))
    return CONS_EVT_RESTART;

  if ((c == 'h') || (c == 'H') || (c == '?'))
    return CONS_EVT_HELP;

  if ((c == 'p') || (c == 'P'))
    return CONS_EVT_PAUSE;

  if ((c == 'x') || (c == 'X'))
    return CONS_EVT_PROF;

  return CONS_EVT_UNDEFINED;
}
#endif

/* -----------------------------------------------------------------------------
 * Exported/Public functions
 * -----------------------------------------------------------------------------
 */

int aiSystemPerformanceInit(void)
{
  int res;
  LC_PRINT("\r\n#\r\n");
  LC_PRINT("# %s %d.%d\r\n", _APP_NAME_ , _APP_VERSION_MAJOR_,
      _APP_VERSION_MINOR_ );
  LC_PRINT("#\r\n");

  systemSettingLog();

  crcIpInit();
  cyclesCounterInit();

  res = aiInit();
  if (res) {
    while (1)
    {
      //HAL_Delay(1000);
      tx_thread_sleep(100);   /* wait 1 Sec */
    }
  }

  srand(3); /* deterministic outcome */

  // test_reloc();
  return 0;
}

int aiSystemPerformanceProcess(void)
{
  int r;

//  do {
    r = aiTestPerformance();
#if 0 
    if (!r) {
      r = aiTestConsole();

      if (r == CONS_EVT_UNDEFINED) {
        r = 0;
      } else if (r == CONS_EVT_HELP) {
        LC_PRINT("\r\n");
        LC_PRINT("Possible key for the interactive console:\r\n");
        LC_PRINT("  [q,Q]      quit the application\r\n");
        LC_PRINT("  [r,R]      re-start (NN de-init and re-init)\r\n");
        LC_PRINT("  [p,P]      pause\r\n");
        LC_PRINT("  [d,D]      hide detailed information ('r' to restore)\r\n");
        LC_PRINT("  [h,H,?]    this information\r\n");
        LC_PRINT("   xx        continue immediately\r\n");
        LC_PRINT("\r\n");
        LC_PRINT("Press any key to continue..\r\n");

        while ((r = aiTestConsole()) == CONS_EVT_TIMEOUT) {
//          HAL_Delay(1000);
          tx_thread_sleep(100);   /* wait 1 Sec */
          
        }
        if (r == CONS_EVT_UNDEFINED)
          r = 0;
      }
      if (r == CONS_EVT_PROF) {
        profiling_mode = true;
        profiling_factor *= 2;
        r = 0;
      }

      if (r == CONS_EVT_HIDE) {
        observer_mode = false;
        r = 0;
      }

      if (r == CONS_EVT_RESTART) {
        profiling_mode = false;
        observer_mode = true;
        profiling_factor = 5;
        LC_PRINT("\r\n");
        aiDeInit();
        aiSystemPerformanceInit();
        r = 0;
      }
      if (r == CONS_EVT_QUIT) {
        profiling_mode = false;
        LC_PRINT("\r\n");
        disableInts();
        aiDeInit();
        LC_PRINT("\r\n");
        LC_PRINT("Board should be reseted...\r\n");
        while (1) {
          //HAL_Delay(1000);
          tx_thread_sleep(100);   /* wait 1 Sec */
        }
      }
      if (r == CONS_EVT_PAUSE) {
        LC_PRINT("\r\n");
        LC_PRINT("Press any key to continue..\r\n");
        while ((r = aiTestConsole()) == CONS_EVT_TIMEOUT) {
          //HAL_Delay(1000);
          tx_thread_sleep(100);   /* wait 1 Sec */
        }
        r = 0;
      }
    }
//  } while (r==0);
#endif
  return r;
}

void aiSystemPerformanceDeInit(void)
{
  LC_PRINT("\r\n");
  aiDeInit();
  LC_PRINT("bye bye ...\r\n");
}

