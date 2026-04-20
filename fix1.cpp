bool SelectorLogic::writeToIKServer(const CartesianInputCommand &cmd) {
    if (!ik_connected_) {
        UA_StatusCode st = UA_Client_connect(ik_client_, config_.ik_endpoint.c_str());
        if (st != UA_STATUSCODE_GOOD) return false;
        ik_connected_ = true;
    }

    // Check write result; if it fails, mark disconnected so we reconnect next tick
    UA_Float v0 = cmd.linear_vel[0];
    UA_Variant val;
    UA_Variant_setScalar(&val, &v0, &UA_TYPES[UA_TYPES_FLOAT]);
    UA_StatusCode st = UA_Client_writeValueAttribute(ik_client_,
        UA_NODEID_NUMERIC(opcua_ids::NS, opcua_ids::CART_VEL_X), &val);

    if (st != UA_STATUSCODE_GOOD) {
        // Connection lost — disconnect and reconnect next tick
        UA_Client_disconnect(ik_client_);
        ik_connected_ = false;
        return false;
    }

    // Connection is alive — write the rest
    writeFloat(ik_client_, opcua_ids::CART_VEL_Y,     cmd.linear_vel[1]);
    writeFloat(ik_client_, opcua_ids::CART_VEL_Z,     cmd.linear_vel[2]);
    writeFloat(ik_client_, opcua_ids::CART_VEL_ROLL,  cmd.angular_vel[0]);
    writeFloat(ik_client_, opcua_ids::CART_VEL_PITCH, cmd.angular_vel[1]);
    writeFloat(ik_client_, opcua_ids::CART_VEL_YAW,   cmd.angular_vel[2]);
    writeFloat(ik_client_, opcua_ids::CART_GRIPPER,   cmd.gripper_cmd);
    writeBool (ik_client_, opcua_ids::CART_DEADMAN,   cmd.deadman);
    writeUInt32(ik_client_, opcua_ids::CART_STAMP,    cmd.stamp);
    return true;
}