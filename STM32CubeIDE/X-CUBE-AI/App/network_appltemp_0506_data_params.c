/**
  ******************************************************************************
  * @file    network_appltemp_0506_data_params.c
  * @author  AST Embedded Analytics Research Platform
  * @date    2025-06-05T10:38:08+0100
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */

#include "network_appltemp_0506_data_params.h"


/**  Activations Section  ****************************************************/
ai_handle g_network_appltemp_0506_activations_table[1 + 2] = {
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
  AI_HANDLE_PTR(NULL),
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
};




/**  Weights Section  ********************************************************/
AI_ALIGNED(32)
const ai_u64 s_network_appltemp_0506_weights_array_u64[21] = {
  0xbea9a56a3e631fd9U, 0xbe381aca3d221ebcU, 0x3da03e5e3e19ed02U, 0xbe16e40cbf28e0c9U,
  0xbf219ee8bea11f8bU, 0xbf06364a3d8b5c70U, 0x3f1f14d7be9b7798U, 0xbecd9d323d621200U,
  0x3c11f1773f49b6c2U, 0xbe69591cbd092550U, 0x3e37fedcbeb1a2c4U, 0x3f0cc0c73e880624U,
  0x3ed7040bbf055832U, 0x0U, 0x3e81bfe4bcec98e6U, 0xbd12d67900000000U,
  0x3f0b9afebf06ae98U, 0xc1a7e3fd409d400eU, 0x3e55fd543e19cf30U, 0xbe31991a41831744U,
  0x40d009d1U,
};


ai_handle g_network_appltemp_0506_weights_table[1 + 2] = {
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
  AI_HANDLE_PTR(s_network_appltemp_0506_weights_array_u64),
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
};

