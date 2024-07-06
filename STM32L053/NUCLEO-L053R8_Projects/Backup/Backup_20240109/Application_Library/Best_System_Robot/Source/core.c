#include "core.h"

/* Initial Configuration---------------------------------------*/
void core_config(void)
{
	servos_init();
	ms_init(MS_V1, false);
	ms_init(MS_L29XX, true);
}

#if !defined(SERVO_H)
/* Math--------------------------------------------------------*/
float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double max(double a, double b)
{
	double q = sqrt((a-b)*(a-b));
	return ((a + b) + q) / 2;
}
#endif

/* Debug--------------------------------------------------------*/
void print_binary(uint8_t byteData) {
	int size = sizeof(byteData)* 8;
	uint8_t bits[size + 1];
	
  for (int i = size - 1; i >= 0; i--) {
    // 判斷 byteData 的第 i 位是 1 還是 0，並將其寫入到 bits 的第 i 個元素
    bits[i] = (byteData & (1 << i)) ? '1' : '0';
	}
	bits[size] = '\0'; //在最後一個元素加上結束符號
	printf("%s\r\n", bits); // 使用 printf 函數，並使用 %s 格式符來輸出 binary
}


/* Robot with JSON--------------------------------------------------------*/
bool json_action(char *JSON_STRING, uint16_t token_size) //sizeof(char)*strlen(JSON_STRING)
{
    cJSON *root = cJSON_ParseWithLength(JSON_STRING, token_size);
    if(root == NULL){
        printf("invalid JSON\r\n"); // Print for debug.
				cJSON_Delete(root); // Free the root pointer
        return false;
    }
		
    for (cJSON *token = root->child;token != NULL;token = token->next){
        if(!strcmp(token->string,"motor")){ // Check if the key is "motor"
            if(cJSON_IsObject(token)){ // Check if the value is an object
							cJSON *motor = token; // Get the "Motor" object
							cJSON *item = NULL; // Declare a pointer to store each item in the object
							cJSON_ArrayForEach(item, motor) { // Loop through all the items in the object
							// Get the key and value of the current item
							char *key = item->string; // The key is the motor number, such as "M1" or "W1"
							float value = item->valuedouble; // The value is the motor input
							//int value = item->valueint; // The value is the motor input, such as -100
							
							// Convert the key to an integer, using the enum type defined in ms_v1_motor_control
							uint8_t dc_motor_number = 0;
							if (strcmp(key, "M1") == 0 || strcmp(key, "W1") == 0) {
									dc_motor_number = M1;
							} else if (strcmp(key, "M2") == 0 || strcmp(key, "W2") == 0) {
									dc_motor_number = M2;
							} else if (strcmp(key, "M3") == 0 || strcmp(key, "W3") == 0) {
									dc_motor_number = M3;
							} else if (strcmp(key, "M4") == 0 || strcmp(key, "W4") == 0) {
									dc_motor_number = M4;
							} else {
									printf("invalid motor number\r\n"); // Print for debug.
									return false; // Invalid motor number
							}
							// Call the ms_motor_control function with the motor shield type, the motor number and input
							if (key[0] == 'M') { // If the key starts with 'M', use MS_V1 type
								ms_motor_control(&motor_shield_v1, MS_V1, dc_motor_number, value);
							} else if (key[0] == 'W') { // If the key starts with 'W', use MS_L29XX type
								ms_motor_control(&motor_shield_l29xx, MS_L29XX, dc_motor_number, value);
							} else {
								printf("invalid motor shield type\r\n"); // Print for debug.
								return false; // Invalid motor shield type
							}
							// print_binary(motor_shield_v1.hc595.byte); // Print for debug.
							}
						}
						else {
							printf("invalid motor value\r\n"); // Print for debug.
							return false; // Invalid motor value
            }
				}

				
				if(!strcmp(token->string,"servo")){ // Check if the key is "servo"
						if(cJSON_IsObject(token)){ // Check if the value is an object
								cJSON *servo_obj = token; // Get the "Servo" object
								cJSON *item = NULL; // Declare a pointer to store each item in the object
								cJSON_ArrayForEach(item, servo_obj) { // Loop through all the items in the object
										// Get the key and value of the current item
										char *key = item->string; // The key is the servo number, such as "S1"
										float value = 0; // Declare a variable to store the servo input
										
										// Convert the key to an integer, using the enum type defined in ms_v1_servo_control
										uint8_t servo_number = 0; // Declare and initialize servo_number at the beginning of the loop
										if (strcmp(key, "S1") == 0) {
												servo_number = S1;
										} else if (strcmp(key, "S2") == 0) {
												servo_number = S2;
										} else if (strcmp(key, "S3") == 0) {
												servo_number = S3;
										} else {
												printf("invalid servo number\r\n"); // Print for debug.
												return false; // Invalid servo number
										}
										
										// Check if the value is a number or a string
										if (cJSON_IsNumber(item)) { // If the value is a number
												value = item->valuedouble; // The value is the pwm value
												// Use servo_control function with PWM mode and limit, and pass the PWM_TypeDef pointer
												servo_control(&servo[servo_number], value, PWM, true); 
										} else if (cJSON_IsString(item)) { // If the value is a string
												char *str = item->valuestring; // The value is the angle string, such as "90 deg" or "stop"
												// Check if the value is "stop"
												if (strcmp(str, "stop") == 0) { // If the value is "stop"
														// Call the pwm_stop function with the PWM_TypeDef pointer
														pwm_stop(&servo[servo_number]);
														continue; // Skip the rest of the loop
												}
												// Parse the string to get the angle value
												char *endptr; // Declare a pointer to store the end of the parsed string
												value = strtof(str, &endptr); // Convert the string to a float
												if (endptr == str) { // If no conversion was performed
													printf("invalid servo value\r\n"); // Print for debug.
													continue; // Skip the rest of the loop
												}
												// No need to check if the string has a "deg" suffix
												// Use servo_control function with ANGLE mode and limit, and pass the PWM_TypeDef pointer
												servo_control(&servo[servo_number], value, ANGLE, true); 
										} else { // If the value is neither a number nor a string
												printf("invalid servo value\r\n"); // Print for debug.
												continue; // Skip the rest of the loop
										}
								}
						}
						else {
							printf("invalid servo value\r\n"); // Print for debug.
							continue; // Skip the rest of the loop
						}
				}

				
				else if(!strcmp(token->string,"74HC595")){
					if (cJSON_IsString(token)) { //檢查 token 是否為字串型態
						printf("74HC595(L->H): "); // Print for debug.
						print_binary(motor_shield_v1.hc595.byte); // Print for debug.
					}
					else {
						int size = cJSON_GetArraySize(token); //取得 array 的元素數量
						uint8_t byteData = 0; //宣告一個 uint8_t 變數，並初始化為 0
						
						for(int i = 0; i < size; i++){ //迴圈遍歷 array 的每個元素
							int value = cJSON_GetArrayItem(token, i)->valueint; //取得第 i 個元素的值
							
							//根據 value 的值，將 byteData 的第 i 位設為 1 或 0
							if (value == 1) {
								byteData |= (1 << i); //使用位元或運算符，將 byteData 的第 i 位設為 1
							}
							else {
								byteData &= ~(1 << i); //使用位元與運算符，將 byteData 的第 i 位設為 0
							}
						}
						HC595_SendByte(&motor_shield_v1.hc595, byteData); //調用 HC595_SendByte 函數，將 byteData 作為參數傳遞
						//print_binary(byteData); // Print for debug.
						//print_binary(motor_shield_v1.hc595.byte); // Print for debug.
					}
        }


        else if(!strcmp(token->string,"adc")){
            //printf("%f\r\n",read_adc()); // send adc value
						//printf("%d\n",(int)map(read_adc(),0,3.26,0,100)); // send adc value by %
						printf("%d\r\n",(int)map((10800000-(float)HAL_GetTick())/10800000,0,1,0,82.5));
        }

    }
    cJSON_Delete(root);
		return true; // JSON action completed successfully
}
