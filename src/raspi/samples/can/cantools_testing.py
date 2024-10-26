import cantools

db = cantools.database.load_file("src/raspi/odrive-cansimple.dbc")

heartbeat_msg = db.get_message_by_name("Heartbeat")
heartbeat_data = {'Trajectory_Done_Flag': 0, 
                    'Controller_Error_Flag': 0, 
                    'Encoder_Error_Flag': 0, 
                    'Motor_Error_Flag': 0,
                    'Axis_State': 3,
                    'Axis_Error': 12}
encoded_data = heartbeat_msg.encode(heartbeat_data)
print(encoded_data)
print(type(heartbeat_msg.decode(encoded_data)['Axis_State']))