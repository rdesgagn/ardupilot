#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_SerialManager/AP_SerialManager_config.h>

#ifndef AP_SCRIPTING_ENABLED
#define AP_SCRIPTING_ENABLED (BOARD_FLASH_SIZE > 1024)
#endif

#if AP_SCRIPTING_ENABLED
    #include <AP_Filesystem/AP_Filesystem_config.h>
    #if !AP_FILESYSTEM_FILE_READING_ENABLED && !defined(HAL_HAVE_AP_ROMFS_EMBEDDED_LUA)
        #error "Scripting requires a filesystem"
    #endif
#endif

#ifndef AP_SCRIPTING_SERIALDEVICE_ENABLED
#define AP_SCRIPTING_SERIALDEVICE_ENABLED AP_SERIALMANAGER_REGISTER_ENABLED && (BOARD_FLASH_SIZE>1024)
#endif
