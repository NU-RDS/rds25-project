#ifndef RDS_CONSTANTS_HPP
#define RDS_CONSTANTS_HPP

/* -------------- Joint ID -------------- */
const int DEX_DIP_ID = 0;
const int DEX_PIP_ID = 1;
const int DEX_MCP_ID = 2;
const int DEX_SPLAIN_ID = 3;
const int POW_GRASP_ID = 4;
const int WRIST_ROLL = 5;
const int WRIST_PITCH = 6;
const int WRIST_YAW = 7;

/* -------------- Tendon ID -------------- */


/* -------------- Range of Motion -------------- */
// Dexterous Finger
const float DEX_DIP_ROM_MAX = 90.0f;  
const float DEX_DIP_ROM_MIN = 0.0f;     
const float DEX_PIP_ROM_MAX = 110.0f;   
const float DEX_PIP_ROM_MIN = 0.0f;   
const float DEX_MCP_ROM_MAX = 90.0f;   
const float DEX_MCP_ROM_MIN = -15.0f;  
const float DEX_SP_ROM_MAX = 15.0f;    
const float DEX_SP_ROM_MIN = -15.0f;   
// Power Finger
const float POW_ROM_MAX = 120.0f;     
const float POW_ROM_MIN = 0.0f;        
// Wrist 
const float WRIST_ROLL_ROM_MAX = 90.0f;  
const float WRIST_ROLL_ROM_MIN = -90.0f; 
const float WRIST_PITCH_ROM_MAX = 60.0f; 
const float WRIST_PITCH_ROM_MIN = -70.0f; 
const float WRIST_YAW_ROM_MAX = 20.0f;   
const float WRIST_YAW_ROM_MIN = -30.0f;  

/* -------------- Mechanical Specs ---------------*/
const float PITCH_RADIUS = 24.0f;
const float YAW_RADIUS = 24.0f;
const float POW_RADIUS = 13.0f;
const float MCP_RADIUS = 7.5f;
const float SPLAY_RADIUS = 7.5f;
const float EXTENSOR_RADIUS = 13.0f;
const float PIP_RADIUS = 18.5f;
const float MOTOR_RADIUS = 5.0f;


#endif 