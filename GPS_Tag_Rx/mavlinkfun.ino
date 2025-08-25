void mavlink_request_comm() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    if ((gotmsg) && (notrequested)) {
      // request streams
//      mavlink_msg_request_data_stream_pack(0, 1, &msg, sysid_from, compid_from, 0, 1, 1);
      mavlink_msg_request_data_stream_pack(1,2, &msg, sysid_from, compid_from, 0, 1, 1);
 // uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop
      
      // Copy the message to the send buffer
      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
 
      // Send the message with the standard UART send function
      mySerial.write(buf, len);

      delay(1000);
      notrequested=0;
    }   
}

void mavlink_set_mode() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint8_t bmode, cmode;

    // Compose message
    //mavlink_msg_request_data_stream_pack(1,2, &msg, sysid_from, compid_from, 0, 1, 1);
    // base-custom values from testing:
    // Auto:    137-10
    // Loiter:  137-5
    // Guided:  137-15
    // RTL:     137-11
    // Hold:    129-4
    // Manual:  193-0
    switch (GPS_mode.mode) 
    {
      case 0: // Manual
        bmode = 193;
        cmode = 0;
        break;
      case 1: // RTL
        bmode = 137;
        cmode = 11;
        break;
      case 2: // Loiter
        bmode = 137;
        cmode = 5;
        break;
      case 3: // Guided
        bmode = 137;
        cmode = 15;
        break;
    }
    mavlink_msg_set_mode_pack(1,2, &msg, 1, bmode, cmode);

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // Send the message with the standard UART send function
    mySerial.write(buf, len);

    if (GPS_mode.mode==3) {
      // Update GPS if in follow mode
      // SET_POSITION_TARGET_GLOBAL_INT https://ardupilot.org/dev/docs/mavlink-rover-commands.html#mavlink-rover-commands-set-position-target-global-int
      mavlink_set_guidepts(GPS_mode.lat, GPS_mode.lon);
    }

}

void mavlink_set_guidepts(double lat, double lon) {
  // Update GPS if in follow mode
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // SET_POSITION_TARGET_GLOBAL_INT https://ardupilot.org/dev/docs/mavlink-rover-commands.html#mavlink-rover-commands-set-position-target-global-int

    // send position to go to also
    //mavlink_msg_set_position_target_global_int_pack(1,2, &msg, millis(), 1, 1, MAV_FRAME_GLOBAL_RELATIVE_ALT, 0x0DFC, 379976940, -1220912560,0,0,0,0,0,0,0,0,0);
    mavlink_msg_set_position_target_global_int_pack(1,2, &msg, millis(), 1, 1, MAV_FRAME_GLOBAL_RELATIVE_ALT, 0x0DFC, lat*1e7, lon*1e7,0,0,0,0,0,0,0,0,0);

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // Send the message with the standard UART send function
    mySerial.write(buf, len);

}

void mavlink_set_2ndveh_inertialpos() {
  // sends inertial position of tag to vehicle
  // GLOBAL_POSITION_INT but with sysid set to new vehicle
  // https://github.com/mavlink/c_library_v1/blob/master/common/mavlink_msg_global_position_int.h 
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_global_position_int_pack(2,1, &msg, millis(), GPS_mode.lat*1e7, GPS_mode.lon*1e7, 0, 0, 0, 0, 0, 0);
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send the message with the standard UART send function
  mySerial.write(buf, len);
}

void mavlink_send_2ndveh_heartbeat() {
// send heartbeat to wake up groundstations??
// https://github.com/mavlink/c_library_v1/blob/master/minimal/mavlink_msg_heartbeat.h
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  //mavlink_msg_global_position_int_pack(2,1, &msg, millis(), GPS_mode.lat*1e7, GPS_mode.lon*1e7, 0, 0, 0, 0, 0, 0);
  uint8_t vehtype = 2; // 11 is boat, 10 is rover, 1 is black airplane, 2 is quadcopter in mission planner
  mavlink_msg_heartbeat_pack(2,1, &msg, vehtype, 3, 137, 11, 4);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send the message with the standard UART send function
  mySerial.write(buf, len);

}

void comm_receive() {
 
  mavlink_message_t msg;
  mavlink_status_t status;
  
  while(mySerial.available() > 0 ) 
  {
    //digitalWrite(D0,(!(digitalRead(D0))));
    uint8_t c = mySerial.read();
    // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      // Handle message
      // see the .h file for each message in C:\Users\Mike\Documents\Arduino\libraries\mavlinkinclude\common
      // \src\mavlink\mavlink\common, \src\mavlink\mavlink\minimal, and all the other folders too
      // there are functions to decode all the sent data, individually or as a data structure.

      // Serial.print ("msg: ");
      // Serial.print(msg.msgid);
      // Serial.print("  SysID: ");
      // Serial.print(msg.sysid);
      // Serial.print("  CMP ID:  ");
      // Serial.println(msg.compid);

      gotmsg=1; // got a message
      sysid_from = msg.sysid; // who it is from
      compid_from = msg.compid;

      if (sysid_from == 1) { // ignore GCS messages
        switch(msg.msgid)
        {
          case MAVLINK_MSG_ID_HEARTBEAT:
          // mavlink_msg_heartbeat.h
                base_mode = mavlink_msg_heartbeat_get_base_mode(&msg);
                custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);  

                if ((base_mode==193)&&(custom_mode==0)) {
                  current_mode = 0; // manual
                }
                if ((base_mode==137)&&(custom_mode==11)) {
                  current_mode = 1; // RTL
                }
                if ((base_mode==137)&&(custom_mode==5)) {
                  current_mode = 2; // Loiter
                }
                if ((base_mode==137)&&(custom_mode==15)) {
                  current_mode = 3; // Guided
                }


                // display.clear();
                // display.drawString(0, 0, "HB:" + String(msg.sysid) + "::" + String(msg.compid) +  "\r\n"+String(base_mode) + ":" + String(custom_mode)); 
                // display.display();
                break;
          case MAVLINK_MSG_ID_COMMAND_LONG:
                // EXECUTE ACTION
                break;
          case MAVLINK_MSG_ID_ATTITUDE:
                // 30
                break;
          case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
                // 46
                // GOT TO WAYPOINT?
                //mavlink_msg_mission_item_reached_decode(&msg, &mission_reached);
                break;
          case MAVLINK_MSG_ID_SYSTEM_TIME:
                // 2
                //timesys_usec = mavlink_msg_system_time_get_time_unix_usec(&msg);
                timesys_usec = mavlink_msg_system_time_get_time_boot_ms(&msg);
                timegps_usec = mavlink_msg_system_time_get_time_unix_usec(&msg);
                break;
          case MAVLINK_MSG_ID_GPS_RAW_INT:
                mavlink_msg_gps_raw_int_decode(&msg, &gps_raw); 
                gotGPS=1;
                break;
          case MAVLINK_MSG_ID_SYS_STATUS:
          // \mavlink\common\mavlink_msg_sys_status.h
                FC_info.volts = (float)mavlink_msg_sys_status_get_voltage_battery(&msg)/1000.0;
                break;
          default:
                //Do nothing
                break;
        }
      }

    }
 
    // And get the next one
  }
}



