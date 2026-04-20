#include "common/skill_types.h"
#include "common/config_loader.h"
#include "common/safety.h"
#include "ik/ik_solver.h"
#include "ik/kinematics.h"

#include <open62541/server.h>
#include <open62541/server_config_default.h>
#include <open62541/client.h>
#include <open62541/client_config_default.h>
#include <open62541/client_highlevel.h>

#include <csignal>
#include <cstdio>
#include <cstring>
#include <chrono>
#include <unistd.h>
#include <Eigen/Dense>

static volatile bool g_running = true;
static void signalHandler(int) { g_running = false; }

static float readLocalFloat(UA_Server *s, uint32_t id) {
    UA_Variant v; UA_Variant_init(&v);
    UA_Server_readValue(s, UA_NODEID_NUMERIC(teleop::opcua_ids::NS, id), &v);
    float r = 0.0f;
    if (v.type == &UA_TYPES[UA_TYPES_FLOAT]) r = *(UA_Float*)v.data;
    UA_Variant_clear(&v);
    return r;
}
static bool readLocalBool(UA_Server *s, uint32_t id) {
    UA_Variant v; UA_Variant_init(&v);
    UA_Server_readValue(s, UA_NODEID_NUMERIC(teleop::opcua_ids::NS, id), &v);
    bool r = false;
    if (v.type == &UA_TYPES[UA_TYPES_BOOLEAN]) r = *(UA_Boolean*)v.data;
    UA_Variant_clear(&v);
    return r;
}
static uint32_t readLocalUInt32(UA_Server *s, uint32_t id) {
    UA_Variant v; UA_Variant_init(&v);
    UA_Server_readValue(s, UA_NODEID_NUMERIC(teleop::opcua_ids::NS, id), &v);
    uint32_t r = 0;
    if (v.type == &UA_TYPES[UA_TYPES_UINT32]) r = *(UA_UInt32*)v.data;
    UA_Variant_clear(&v);
    return r;
}

static float clientReadFloat(UA_Client *c, uint32_t id) {
    UA_Variant v; UA_Variant_init(&v);
    UA_Client_readValueAttribute(c, UA_NODEID_NUMERIC(teleop::opcua_ids::NS, id), &v);
    float r = 0.0f;
    if (v.type == &UA_TYPES[UA_TYPES_FLOAT]) r = *(UA_Float*)v.data;
    UA_Variant_clear(&v);
    return r;
}
static void clientWriteFloat(UA_Client *c, uint32_t id, float v) {
    UA_Variant val; UA_Variant_setScalar(&val, &v, &UA_TYPES[UA_TYPES_FLOAT]);
    UA_Client_writeValueAttribute(c, UA_NODEID_NUMERIC(teleop::opcua_ids::NS, id), &val);
}
static void clientWriteBool(UA_Client *c, uint32_t id, bool v) {
    UA_Boolean b = v; UA_Variant val;
    UA_Variant_setScalar(&val, &b, &UA_TYPES[UA_TYPES_BOOLEAN]);
    UA_Client_writeValueAttribute(c, UA_NODEID_NUMERIC(teleop::opcua_ids::NS, id), &val);
}
static void clientWriteUInt32(UA_Client *c, uint32_t id, uint32_t v) {
    UA_UInt32 x = v; UA_Variant val;
    UA_Variant_setScalar(&val, &x, &UA_TYPES[UA_TYPES_UINT32]);
    UA_Client_writeValueAttribute(c, UA_NODEID_NUMERIC(teleop::opcua_ids::NS, id), &val);
}

int main(int argc, char *argv[]) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::string config_path = "configs/ik.yaml";
    if (argc > 1) config_path = argv[1];

    printf("═══════════════════════════════════════════\n");
    printf("  OPC UA IK Server (Cartesian → Joint) v2\n");
    printf("═══════════════════════════════════════════\n");

    teleop::IKConfig config;
    try { config = teleop::loadIKConfig(config_path); }
    catch (const std::exception &e) { printf("ERROR: %s\n", e.what()); return 1; }

    UA_Server *server = UA_Server_new();
    UA_ServerConfig *sc = UA_Server_getConfig(server);
    UA_ServerConfig_setMinimal(sc, config.opcua_port, NULL);
    sc->applicationDescription.applicationName =
        UA_LOCALIZEDTEXT_ALLOC("en-US", "IKServer");

    if (teleop::addCartesianInputSkillNodes(server) != UA_STATUSCODE_GOOD) {
        printf("ERROR: add nodes failed\n"); return 1;
    }
    if (UA_Server_run_startup(server) != UA_STATUSCODE_GOOD) {
        printf("ERROR: startup failed\n"); return 1;
    }
    printf("[IK] OPC UA server on port %d\n", config.opcua_port);

    UA_Client *fb_client = UA_Client_new();
    UA_ClientConfig_setDefault(UA_Client_getConfig(fb_client));
    bool fb_connected = false;

    UA_Client *bridge_client = UA_Client_new();
    UA_ClientConfig_setDefault(UA_Client_getConfig(bridge_client));
    bool bridge_connected = false;

    teleop::ik::IKParams ik_params;
    ik_params.max_iterations = config.max_iterations;
    ik_params.eps_pos = config.eps_pos;
    ik_params.eps_ori = config.eps_ori;
    ik_params.damping_lambda = config.damping_lambda;
    ik_params.jl_weight = config.jl_weight;
    auto limits = teleop::ik::JointLimitsEigen::vx300();

    Eigen::Matrix<double, 5, 1> current_joints; current_joints.setZero();
    Eigen::Matrix4d T_target;
    bool have_initial_pose = false;

    uint32_t last_stamp = 0;
    auto last_cmd_time = std::chrono::steady_clock::now();

    printf("[IK] Running at 50 Hz.\n\n");
    int period_us = 20000;
    uint32_t loop_count = 0;

    while (g_running) {
        UA_Server_run_iterate(server, false);

        if (!fb_connected) {
            if (UA_Client_connect(fb_client, config.feedback_endpoint.c_str())
                == UA_STATUSCODE_GOOD) {
                fb_connected = true;
                printf("[IK] fb server connected\n");
            }
        }
        if (!bridge_connected) {
            if (UA_Client_connect(bridge_client, config.bridge_endpoint.c_str())
                == UA_STATUSCODE_GOOD) {
                bridge_connected = true;
                printf("[IK] bridge connected\n");
            }
        }

        if (fb_connected) {
            for (int i = 0; i < 5; i++) {
                current_joints(i) = (double)clientReadFloat(fb_client,
                    teleop::opcua_ids::FB_JOINT_POS_BASE + i);
            }
        }

        teleop::CartesianInputCommand cmd;
        cmd.linear_vel[0]  = readLocalFloat(server, teleop::opcua_ids::CART_VEL_X);
        cmd.linear_vel[1]  = readLocalFloat(server, teleop::opcua_ids::CART_VEL_Y);
        cmd.linear_vel[2]  = readLocalFloat(server, teleop::opcua_ids::CART_VEL_Z);
        cmd.angular_vel[0] = readLocalFloat(server, teleop::opcua_ids::CART_VEL_ROLL);
        cmd.angular_vel[1] = readLocalFloat(server, teleop::opcua_ids::CART_VEL_PITCH);
        cmd.angular_vel[2] = readLocalFloat(server, teleop::opcua_ids::CART_VEL_YAW);
        cmd.gripper_cmd    = readLocalFloat(server, teleop::opcua_ids::CART_GRIPPER);
        cmd.deadman        = readLocalBool(server, teleop::opcua_ids::CART_DEADMAN);
        cmd.stamp          = readLocalUInt32(server, teleop::opcua_ids::CART_STAMP);

        // SIMPLIFIED freshness: always treat as fresh if there's motion OR deadman
        float motion_mag = std::abs(cmd.linear_vel[0]) + std::abs(cmd.linear_vel[1])
                         + std::abs(cmd.linear_vel[2]) + std::abs(cmd.angular_vel[0])
                         + std::abs(cmd.angular_vel[1]) + std::abs(cmd.angular_vel[2]);
        bool fresh = (motion_mag > 0.01f) || cmd.deadman;

        if (!have_initial_pose && fb_connected) {
            T_target = teleop::ik::fkInSpace(
                teleop::ik::getM(), teleop::ik::getSlist(), current_joints);
            have_initial_pose = true;
            printf("[IK] Initial pose set from FK\n");
        }

        Eigen::Matrix<double, 6, 1> V_space;
        V_space.setZero();
        if (have_initial_pose && motion_mag > 0.01f) {
            V_space(0) = cmd.angular_vel[0] * config.angular_vel_scale;
            V_space(1) = cmd.angular_vel[1] * config.angular_vel_scale;
            V_space(2) = cmd.angular_vel[2] * config.angular_vel_scale;
            V_space(3) = cmd.linear_vel[0]  * config.linear_vel_scale;
            V_space(4) = cmd.linear_vel[1]  * config.linear_vel_scale;
            V_space(5) = cmd.linear_vel[2]  * config.linear_vel_scale;
        }

        double dt = 0.02;
        Eigen::Matrix4d T_target_new = T_target;
        if (have_initial_pose && V_space.norm() > 1e-6) {
            T_target_new = teleop::ik::integrateTwist(T_target, V_space, dt);
        }

        // Solve IK
        Eigen::Matrix<double, 5, 1> new_joints = current_joints;
        bool ik_ok = false;
        if (have_initial_pose && motion_mag > 0.01f) {
            ik_ok = teleop::ik::solveIK(T_target_new, current_joints,
                                         ik_params, limits, new_joints);
            if (ik_ok) {
                T_target = T_target_new;  // advance target only on success
            }
            // If IK fails we STILL use new_joints (best-effort result)
        }

        // Compute velocity from joint delta
        const float BRIDGE_MAX_RAD_S = 1.0f;
        float joint_vels_normalized[5];
        for (int i = 0; i < 5; i++) {
            double raw_vel = (new_joints(i) - current_joints(i)) / dt;
            float n = (float)(raw_vel / BRIDGE_MAX_RAD_S);
            if (n > 1.0f) n = 1.0f;
            if (n < -1.0f) n = -1.0f;
            joint_vels_normalized[i] = n;
        }

        if (bridge_connected) {
            clientWriteFloat(bridge_client, teleop::opcua_ids::JOINT_VEL_WAIST,
                             joint_vels_normalized[0]);
            clientWriteFloat(bridge_client, teleop::opcua_ids::JOINT_VEL_SHOULDER,
                             joint_vels_normalized[1]);
            clientWriteFloat(bridge_client, teleop::opcua_ids::JOINT_VEL_ELBOW,
                             joint_vels_normalized[2]);
            clientWriteFloat(bridge_client, teleop::opcua_ids::JOINT_VEL_WRIST_ANGLE,
                             joint_vels_normalized[3]);
            clientWriteFloat(bridge_client, teleop::opcua_ids::JOINT_VEL_WRIST_ROT,
                             joint_vels_normalized[4]);
            clientWriteFloat(bridge_client, teleop::opcua_ids::GRIPPER_CMD,
                             cmd.gripper_cmd);
            clientWriteBool(bridge_client, teleop::opcua_ids::DEADMAN, true);  // always true
            clientWriteUInt32(bridge_client, teleop::opcua_ids::STAMP,
                              loop_count);  // always incrementing
        }

        if (++loop_count % 25 == 0) {
            printf("[IK] cart=(x%.2f,y%.2f,z%.2f) motion=%.2f dm=%d | "
                   "jvel=[%.2f,%.2f,%.2f,%.2f,%.2f] ik_ok=%d\n",
                   cmd.linear_vel[0], cmd.linear_vel[1], cmd.linear_vel[2],
                   motion_mag, cmd.deadman,
                   joint_vels_normalized[0], joint_vels_normalized[1],
                   joint_vels_normalized[2], joint_vels_normalized[3],
                   joint_vels_normalized[4], ik_ok);
        }

        usleep(period_us);
    }

    printf("\n[IK] Shutting down...\n");
    if (fb_connected) UA_Client_disconnect(fb_client);
    UA_Client_delete(fb_client);
    if (bridge_connected) UA_Client_disconnect(bridge_client);
    UA_Client_delete(bridge_client);
    UA_Server_run_shutdown(server);
    UA_Server_delete(server);
    return 0;
}