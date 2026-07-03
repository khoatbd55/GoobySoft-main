#include "J1939EngineSimulatorDialog.h"
#include "../../../../Tools/Tools.h"

// State variables for the J1939 Engine Simulator
static bool simulator_active  = false;
static float coolant_temp     = 85.0f;    // SPN 110: Engine Coolant Temperature (default: 85 °C)
static float fuel_temp        = 40.0f;    // SPN 174: Engine Fuel Temperature 1 (default: 40 °C)
static float oil_temp         = 90.0f;    // SPN 175: Engine Oil Temperature 1 (default: 90 °C)
static float oil_pressure     = 400.0f;   // SPN 100: Engine Oil Pressure (default: 400 kPa)
static float engine_speed     = 1500.0f;  // SPN 190: Engine Speed (default: 1500 rpm)
static float engine_torque    = 50.0f;    // SPN 513: Actual Engine Percent Torque (default: 50%)
static int   send_interval_ms = 500;      // Full cycle period (default: 500 ms)
static int   msg_gap_ms       = 50;       // Gap between each frame (fixed 50 ms)
static uint64_t last_send_time = 0;
static int send_state          = 0;       // 0=EEC1, 1=ET1, 2=EFL_P1
static int total_messages_sent = 0;

// Received J1939 control commands
static float commanded_speed          = -1.0f;   // SPN 898: Engine Requested Speed/Speed Limit (default: -1.0f)
static bool commanded_speed_received  = false;
static uint64_t last_commanded_speed_time = 0;

void Windows_Dialogs_AnalyzeDialogs_J1939EngineSimulatorDialog_showJ1939EngineSimulatorDialog(bool* j1939EngineSimulatorDialog) {
    if (ImGui::Begin("J1939 Engine Simulator", j1939EngineSimulatorDialog, ImGuiWindowFlags_AlwaysAutoResize)) {
        
        ImGui::Text("Simulate engine parameters over standard SAE J1939 PGNs");
        ImGui::Separator();
        
        // Control & Configuration
        ImGui::Checkbox("Simulator Active", &simulator_active);
        ImGui::SliderInt("Cycle Interval (ms)", &send_interval_ms, 150, 5000);
        ImGui::Text("Frame gap: 50 ms fixed (EEC1 -> ET1 -> EFL_P1)");
        
        ImGui::Separator();
        ImGui::Text("Engine Parameters:");
        
        // Engine Speed (SPN 190): 0 to 8000 rpm
        ImGui::SliderFloat("Engine Speed (rpm)", &engine_speed, 0.0f, 8000.0f, "%.0f rpm");
        
        // Engine Torque (SPN 513): -125% to 125%
        ImGui::SliderFloat("Engine Torque (%%)", &engine_torque, -125.0f, 125.0f, "%.1f %%");
        
        // Coolant Temp (SPN 110): -40 °C to 210 °C
        ImGui::SliderFloat("Coolant Temperature (C)", &coolant_temp, -40.0f, 210.0f, "%.1f C");
        
        // Fuel Temp (SPN 174): -40 °C to 210 °C
        ImGui::SliderFloat("Fuel Temperature (C)", &fuel_temp, -40.0f, 210.0f, "%.1f C");
        
        // Oil Temp (SPN 175): -40 °C to 150 °C
        ImGui::SliderFloat("Oil Temperature (C)", &oil_temp, -40.0f, 150.0f, "%.1f C");
        
        // Oil Pressure (SPN 100): 0 to 1000 kPa
        ImGui::SliderFloat("Oil Pressure (kPa)", &oil_pressure, 0.0f, 1000.0f, "%.1f kPa");
        
        ImGui::Separator();
        ImGui::Text("Statistics:");
        ImGui::Text("Total Sent J1939 CAN Frames: %d", total_messages_sent);
        const char* state_names[] = { "EEC1 (Speed/Torque)", "ET1 (Temperature)", "EFL_P1 (Pressure)" };
        ImGui::Text("Next frame: %s", state_names[send_state]);
        
        ImGui::Separator();
        ImGui::Text("J1939 Transmitted Signals Monitor:");
        
        // Highlight Engine Speed (SPN 190)
        ImGui::Text("Engine Speed (SPN 190):");
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "%.2f rpm", engine_speed);
        ImGui::SameLine();
        {
            int raw_speed = (int)(engine_speed * 8.0f);
            if (raw_speed < 0)     raw_speed = 0;
            if (raw_speed > 64255) raw_speed = 64255;
            ImGui::Text("(Raw: 0x%04X, Bytes: [%02X %02X])", raw_speed, raw_speed & 0xFF, (raw_speed >> 8) & 0xFF);
        }

        // Display other parameters for completeness
        ImGui::Text("Engine Torque (SPN 513): %.1f %% (Raw: 0x%02X)", engine_torque, (int)(engine_torque + 125.0f) & 0xFF);
        ImGui::Text("Coolant Temp (SPN 110): %.1f C (Raw: 0x%02X)", coolant_temp, (int)(coolant_temp + 40.0f) & 0xFF);
        ImGui::Text("Fuel Temp (SPN 174): %.1f C (Raw: 0x%02X)", fuel_temp, (int)(fuel_temp + 40.0f) & 0xFF);
        ImGui::Text("Oil Temp (SPN 175): %.1f C (Raw: 0x%04X)", oil_temp, (int)((oil_temp + 273.0f) * 32.0f) & 0xFFFF);
        ImGui::Text("Oil Pressure (SPN 100): %.1f kPa (Raw: 0x%02X)", oil_pressure, (int)(oil_pressure / 4.0f) & 0xFF);

        ImGui::Separator();
        ImGui::Text("Received J1939 Speed (from Bus):");
        if (commanded_speed_received) {
            ImGui::Text("Engine Speed (TSC1/EEC1):");
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "%.2f rpm", commanded_speed);
            ImGui::SameLine();
            uint64_t elapsed_ms = SDL_GetTicks() - last_commanded_speed_time;
            ImGui::TextDisabled(" (Last: %.1fs ago)", elapsed_ms / 1000.0f);
        } else {
            ImGui::TextDisabled("No speed command (TSC1/EEC1) received.");
        }

        ImGui::Separator();
        if (ImGui::Button("Reset Stats")) {
            total_messages_sent = 0;
            send_state = 0;
            commanded_speed_received = false;
            commanded_speed = -1.0f;
        }
        
        ImGui::End();
    }
}

void Windows_Dialogs_AnalyzeDialogs_J1939EngineSimulatorDialog_update() {
    // Listen for incoming J1939 speed commands
    J1939* j1939 = Tools_Hardware_ParameterStore_getJ1939();
    if (j1939 && j1939->ID_and_data_is_updated) {
        j1939->ID_and_data_is_updated = false;
        
        uint32_t ID = j1939->ID;
        uint8_t* data = j1939->data;
        uint8_t id1 = (ID >> 16) & 0xFF; // PDU Format (PF)
        
        // Option 1: PGN 0 (TSC1 - Requested Speed Control)
        if (id1 == 0) {
            uint16_t raw_req_speed = ((uint16_t)data[2] << 8) | data[1];
            commanded_speed = raw_req_speed * 0.125f;
            commanded_speed_received = true;
            last_commanded_speed_time = SDL_GetTicks();
        }
        // Option 2: PGN 61444 (0xF004 - EEC1 Actual Engine Speed) from external node (Source Address != 0x00)
        else if (id1 == 240 && (ID >> 8 & 0xFF) == 4 && (ID & 0xFF) != 0x00) {
            // Support both standard Byte 4-5 (data[3], data[4]) and STM32 example's Byte 5-6 (data[4], data[5])
            uint16_t raw_speed_std = ((uint16_t)data[4] << 8) | data[3];
            uint16_t raw_speed_custom = ((uint16_t)data[5] << 8) | data[4];
            
            float speed_val = 0.0f;
            if (raw_speed_std != 0xFFFF) {
                speed_val = raw_speed_std * 0.125f;
            } else {
                speed_val = raw_speed_custom * 0.125f;
            }
            
            commanded_speed = speed_val;
            commanded_speed_received = true;
            last_commanded_speed_time = SDL_GetTicks();
        }
    }

    if (!simulator_active) {
        return;
    }
    
    uint64_t current_time = SDL_GetTicks();
    
    // Determine the interval to wait depending on state:
    // - States 0,1,2 fire at 50 ms gap each
    // - After state 2 (last frame), wait remaining time until next full cycle
    uint64_t wait_time;
    if (send_state < 2) {
        // Between consecutive frames: fixed 50 ms gap
        wait_time = (uint64_t)msg_gap_ms;
    } else {
        // After the last frame: wait the rest of the cycle
        // Full cycle = send_interval_ms, already spent 2*50ms = 100ms
        uint64_t spent = (uint64_t)(msg_gap_ms * 2);
        wait_time = (send_interval_ms > (int)spent) ? (uint64_t)send_interval_ms - spent : (uint64_t)msg_gap_ms;
    }

    if (current_time - last_send_time < wait_time) {
        return;
    }
    last_send_time = current_time;

    if (send_state == 0) {
        // --------------------------------------------------------
        // Frame 1: EEC1 - PGN 61444 (0xF004) - Speed & Torque
        //          CAN ID 0x0CF00400 (Priority 3, SA 0x00)
        // --------------------------------------------------------
        uint8_t eec1_data[8];
        
        // Byte 1: Engine Torque Mode - not available
        eec1_data[0] = 0xFF;
        
        // Byte 2: Driver's Demand Percent Torque (1%/bit, -125% offset)
        int raw_torque_demand = (int)(engine_torque + 125.0f);
        if (raw_torque_demand < 0)   raw_torque_demand = 0;
        if (raw_torque_demand > 250) raw_torque_demand = 250;
        eec1_data[1] = (uint8_t)raw_torque_demand;
        
        // Byte 3: Actual Engine Percent Torque (1%/bit, -125% offset)
        int raw_torque = (int)(engine_torque + 125.0f);
        if (raw_torque < 0)   raw_torque = 0;
        if (raw_torque > 250) raw_torque = 250;
        eec1_data[2] = (uint8_t)raw_torque;
        
        // Byte 4-5: Engine Speed SPN 190 (0.125 rpm/bit, LSB first)
        int raw_speed = (int)(engine_speed * 8.0f);
        if (raw_speed < 0)     raw_speed = 0;
        if (raw_speed > 64255) raw_speed = 64255;
        eec1_data[3] = raw_speed & 0xFF;
        eec1_data[4] = (raw_speed >> 8) & 0xFF;
        
        // Byte 6-8: Not available
        eec1_data[5] = 0xFF;
        eec1_data[6] = 0xFF;
        eec1_data[7] = 0xFF;
        
        CAN_Send_Message(0x0CF00400, eec1_data);
        total_messages_sent++;
        send_state = 1;
    }
    else if (send_state == 1) {
        // --------------------------------------------------------
        // Frame 2: ET1 - PGN 65262 (0xFEEE) - Temperature
        //          CAN ID 0x18FEEE00 (Priority 6, SA 0x00)
        // --------------------------------------------------------
        uint8_t et1_data[8];
        
        // Byte 1: Coolant Temp (1°C/bit, -40°C offset)
        int raw_coolant = (int)(coolant_temp + 40.0f);
        if (raw_coolant < 0)   raw_coolant = 0;
        if (raw_coolant > 250) raw_coolant = 250;
        et1_data[0] = (uint8_t)raw_coolant;
        
        // Byte 2: Fuel Temp (1°C/bit, -40°C offset)
        int raw_fuel = (int)(fuel_temp + 40.0f);
        if (raw_fuel < 0)   raw_fuel = 0;
        if (raw_fuel > 250) raw_fuel = 250;
        et1_data[1] = (uint8_t)raw_fuel;
        
        // Byte 3-4: Oil Temp (0.03125°C/bit = 1/32, -273°C offset, 16-bit LSB first)
        int raw_oil = (int)((oil_temp + 273.0f) * 32.0f);
        if (raw_oil < 0)     raw_oil = 0;
        if (raw_oil > 64255) raw_oil = 64255;
        et1_data[2] = raw_oil & 0xFF;
        et1_data[3] = (raw_oil >> 8) & 0xFF;
        
        // Byte 5-8: Not available
        et1_data[4] = 0xFF;
        et1_data[5] = 0xFF;
        et1_data[6] = 0xFF;
        et1_data[7] = 0xFF;
        
        CAN_Send_Message(0x18FEEE00, et1_data);
        total_messages_sent++;
        send_state = 2;
    }
    else {
        // --------------------------------------------------------
        // Frame 3: EFL_P1 - PGN 65263 (0xFEEF) - Oil Pressure
        //          CAN ID 0x18FEEF00 (Priority 6, SA 0x00)
        // --------------------------------------------------------
        uint8_t efl_data[8];
        efl_data[0] = 0xFF;  // Byte 1: Fuel Delivery Pressure - N/A
        efl_data[1] = 0xFF;  // Byte 2: Engine Oil Filter Differential Pressure - N/A
        efl_data[2] = 0xFF;  // Byte 3: Engine Oil Level - N/A
        
        // Byte 4: Engine Oil Pressure (4 kPa/bit, 0 offset)
        int raw_pressure = (int)(oil_pressure / 4.0f);
        if (raw_pressure < 0)   raw_pressure = 0;
        if (raw_pressure > 250) raw_pressure = 250;
        efl_data[3] = (uint8_t)raw_pressure;
        
        // Byte 5-8: Not available
        efl_data[4] = 0xFF;
        efl_data[5] = 0xFF;
        efl_data[6] = 0xFF;
        efl_data[7] = 0xFF;
        
        CAN_Send_Message(0x18FEEF00, efl_data);
        total_messages_sent++;
        send_state = 0;  // Reset to beginning of next cycle
    }
}
