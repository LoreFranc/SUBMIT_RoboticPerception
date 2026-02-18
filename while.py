print("Waiting for data...")
    
    


    while True:
        try:
            # 3. Receive the message
            # MADS sends multipart messages: [Topic, JSON_Payload]
            msg = socket.recv_multipart()

            payload = msg[1]  # JSON payload is the second part


            try:
                topic = msg[0].decode('utf-8')
            except:
                # If topic is not decodable, skip this message
                continue
            if topic not in TOPIC_FILTER:
                # If topic is not in our filter, skip decoding the payload
                continue

            data = None

            # --- STRATEGIA DI DECODIFICA IBRIDA ---
            
            # TENTATIVO 1: JSON "Sporco" (Trova la prima '{')
            # Questo risolve il problema del tuo log attuale (b'\x01\xf0O{"agent_id"...)
            try:
                # Cerchiamo l'inizio del JSON
                start_idx = payload.find(b'{')
                end_idx = payload.rfind(b'}')
                
                if start_idx != -1 and end_idx != -1 and end_idx > start_idx:
                    # Buttiamo via l'header binario e decodifichiamo il resto
                    json_bytes = payload[start_idx : end_idx+1]
                    json_str = json_bytes.decode('utf-8', errors='ignore')
                    data = json.loads(json_str)
                else:
                    # Se non c'è '{', proviamo MessagePack come fallback
                    raise ValueError("No JSON start found")

            except Exception as e:

                try:
                    unpacker = msgpack.Unpacker(raw=True, strict_map_key=False) 
                    unpacker.feed(payload)

                    data_raw = next(unpacker)  # Get the first unpacked object
                except:
                    continue


            if not isinstance(data, dict):
                continue


            # --- DEBUG BRUTALE ---
            if "pose" in data:
                print("\n[DEBUG] POSE trovata!")
                
                if "debug" in data:
                    print(f"[DEBUG] DEBUG trovato! Chiavi: {list(data['debug'].keys())}")
                    # Controlliamo se ci sono dei NaN che rendono invisibile il grafico
                    raw = data["debug"].get("raw_encoder_only")
                    print(f"[DEBUG] Valore Raw Encoder: {raw}")
                else:
                    print("[ALLARME] Chiave 'debug' ASSENTE nel dizionario!")
            # ---------------------

            # 4. Extract data (with error handling)
            time_val = data.get("sim_time")
            if time_val is None: time_val = get_nested(data, "timecode")
            #if time_val is not None: 
                #rr.set_time_seconds("sim_time", float(time_val))
           
            # Odometry Filter Trajectory - RED
            if topic == "odometry_filter":
                
                if "pose" in data:
                     # FULL EKF PURA        
                    pos = data["pose"]["position"]
                    # Usiamo un nome univoco per la scia raw
                    if not hasattr(main, "traj_pos"): main.traj_pos = []
                    main.traj_pos.append(pos)                    
                    rr.log("robot/FULL_EKF_body", rr.Points3D([pos], radii=0.04, colors=[255, 255, 0], labels="EKF"))
                    rr.log("robot/FULL_EKF_path", rr.LineStrips3D([main.traj_pos], colors=[[255, 255, 0]], radii=0.01)) #sistema colore
                    
                # NUOVO: Grafico Comparativo Angoli (Chi comanda?)
                if "debug" in data:
                    th_enc = data["debug"]["angles"]["theta_enc"]
                    th_imu = data["debug"]["angles"]["theta_imu"]
                    th_ratio = data["debug"].get("angle_ratio")
                    if th_ratio is not None: rr.log("debug/slip/angle_ratio", Scalars(th_ratio))
                    accel_ratio = data["debug"].get("accel_ratio")
                    if accel_ratio is not None: rr.log("debug/slip/accel_ratio", Scalars(accel_ratio))
                    ds = data["debug"].get("ds")
                    ds_angle = data["debug"].get("ds_angle")
                    ds_accel = data["debug"].get("ds_accel")
                    ds_final = data["debug"].get("ds_final")
                    if ds is not None: rr.log("debug/slip/ds", Scalars(ds))
                    if ds_angle is not None: rr.log("debug/slip/ds_angle", Scalars(ds_angle))
                    if ds_accel is not None: rr.log("debug/slip/ds_accel", Scalars(ds_accel))
                    if ds_final is not None: rr.log("debug/slip/ds_final", Scalars(ds_final))
                    
                    gyro_z = data["debug"].get("current_gyro_z")
                    if gyro_z is not None: rr.log("debug/slip/current_gyro_z", Scalars(gyro_z))

                    if th_enc is not None: rr.log("debug/angles/theta_encoder", Scalars(th_enc))
                    if th_imu is not None: rr.log("debug/angles/theta_imu", Scalars(th_imu))

                    if "fused_velocity" in data["debug"]:
                        rr.log("fusion/debug/velocity_fused", Scalars(data["debug"]["fused_velocity"]))
                
                    # ODOMETRIA PURA        
                    raw_pos = data["debug"]["raw_encoder_only"]
                    # Usiamo un nome univoco per la scia raw
                    if not hasattr(main, "traj_raw_enc"): main.traj_raw_enc = []
                    main.traj_raw_enc.append(raw_pos)                    
                    rr.log("robot/raw_encoder_body", rr.Points3D([raw_pos], radii=0.04, colors=[255, 0, 0], labels="Odometry"))
                    rr.log("robot/raw_encoder_path", rr.LineStrips3D([main.traj_raw_enc], colors=[[255, 0, 0]], radii=0.01))

                     # Odom corrected        
                    partial_pos = data["debug"]["Odom_corrected"]
                    # Usiamo un nome univoco per la scia raw
                    if not hasattr(main, "traj_partial_ekf"): main.traj_partial_ekf = []
                    main.traj_partial_ekf.append(partial_pos)                    
                    rr.log("robot/Odom_corrected_body", rr.Points3D([partial_pos], radii=0.04, colors=[0, 255, 0], labels="Odom corrected"))
                    rr.log("robot/Odom_corrected_path", rr.LineStrips3D([main.traj_partial_ekf], colors=[[0, 255, 0]], radii=0.01))

                     # Aruco data      
                    rs_center = data["debug"]["rs_center"]
                    if not hasattr(main, "traj_rs_center"): main.traj_rs_center = []
                    main.traj_rs_center.append(rs_center)
                    rr.log("robot/rs_center_path", rr.LineStrips3D([main.traj_rs_center], colors=[[0, 255, 255]], radii=0.01))
                    rr.log("robot/rs_center_debug", rr.LineStrips3D([[rs_center[0], rs_center[1], 0.0]], colors=[[0, 255, 255]], radii=0.02, labels="RS Center Path"))

                    
                    # Htc position
                    if "htc_position" in data["debug"]:
                        htc_pos = data["debug"]["htc_position"]
                        if not hasattr(main, "traj_htc_position"): main.traj_htc_position = []
                        main.traj_htc_position.append(htc_pos)
                        rr.log("robot/htc_position_path", rr.LineStrips3D([main.traj_htc_position], colors=[[0, 255, 0]], radii=0.01))
                        rr.log("robot/htc_position_debug", rr.Points3D([htc_pos], colors=[[0, 255, 0]], radii=0.02, labels="HTC"))
                        htc_angle = data["debug"]["angles"]["htc_angle"]
                        if htc_angle is not None:
                            rr.log("debug/angles/htc_angle", Scalars(htc_angle))
                    


                    # Acceleration from enc vs imu
                    acc_enc = data["debug"].get("accel_enc")
                    acc_imu = data["debug"].get("accel_imu")
                    is_slipping = data["debug"].get("is_slipping")
                    if acc_enc is not None: rr.log("debug/slip/accel_encoder", Scalars(acc_enc))
                    if acc_imu is not None: rr.log("debug/slip/accel_imu", Scalars(acc_imu))
                    if is_slipping is not None: rr.log("debug/slip/is_slipping_flag", Scalars(is_slipping))

                    # angles
                    ang_rs = get_nested(data, "/debug/angles/theta_rs")
                    ang_fused = get_nested(data, "/debug/angles/fused_full")
                    ang_fused_partial = get_nested(data, "/debug/angles/fused_partial")
                    if ang_rs is not None: rr.log("debug/angles/theta_rs", Scalars(ang_rs))
                    if ang_fused is not None: rr.log("debug/angles/fused_full", Scalars(ang_fused))
                    if ang_fused_partial is not None: rr.log("debug/angles/fused_partial", Scalars(ang_fused_partial))
                

                if "evaluation" in data:
                    err_dist = data["evaluation"].get("current_error_distance")
                    err_theta = data["evaluation"].get("current_error_theta")
                    rmse_dist = data["evaluation"].get("rmse_dist")
                    rmse_theta = data["evaluation"].get("rmse_theta")
                    std_dist = data["evaluation"].get("std_dist")

                    if err_dist is not None: rr.log("evaluation/error_distance", Scalars(err_dist))
                    if err_theta is not None: rr.log("evaluation/error_theta", Scalars(err_theta))
                    if rmse_dist is not None: rr.log("evaluation/rmse_distance", Scalars(rmse_dist))
                    if rmse_theta is not None: rr.log("evaluation/rmse_theta", Scalars(rmse_theta))
                    if std_dist is not None: rr.log("evaluation/std_distance", Scalars(std_dist))
                

        except KeyboardInterrupt:
            print("Manual interruption.")
            break
        except Exception as e:
            print(f"Parsing error: {e}")