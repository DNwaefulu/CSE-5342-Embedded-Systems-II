/* Common Terminal Interface Library
 * Sarker Nadir Afridi Azmi
 *
 * This file contains all the defines, structures, and prototypes that are required for the
 * common terminal interface to operate.
 *
 */

#ifndef COMMON_TERMINAL_INTERFACE_H_
#define COMMON_TERMINAL_INTERFACE_H_

#define MAX_CHARS 80
#define MAX_FIELDS 5

//-----------------------------------------------------------------------------
// Structs
//-----------------------------------------------------------------------------
typedef struct _USER_DATA
{
    char buffer[MAX_CHARS + 1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
} USER_DATA;

bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments);
int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber);
char* getFieldString(USER_DATA* data, uint8_t fieldNumber);
void parseField(USER_DATA* data);
void getsUart0(USER_DATA* data);
void shell(void);

#endif /* COMMON_TERMINAL_INTERFACE_H_ */
