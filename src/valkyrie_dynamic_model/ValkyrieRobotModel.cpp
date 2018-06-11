#include "ValkyrieRobotModel.hpp"
#include "Valkyrie_Dyn_Model.hpp"
#include "Valkyrie_Kin_Model.hpp"
#include "rbdl/urdfreader.h"
#include "Utils/utilities.hpp"

#include <stdio.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

ValkyrieRobotModel* ValkyrieRobotModel::GetValkyrieRobotModel(){
    static ValkyrieRobotModel valkyrie_model_;
    return & valkyrie_model_;
}

ValkyrieRobotModel::ValkyrieRobotModel(){
    model_ = new Model();

    if (!Addons::URDFReadFromFile (URDF_PATH"r5_urdf_rbdl.urdf", model_, false)) {
        std::cerr << "Error loading model ./r5_urdf_rbdl.urdf" << std::endl;
        abort();
    }

    dyn_model_ = new Valkyrie_Dyn_Model(model_);
    kin_model_ = new Valkyrie_Kin_Model(model_);

    printf("[Valkyrie Model] Contructed\n");
}

ValkyrieRobotModel::~ValkyrieRobotModel(){
    delete dyn_model_;
    delete kin_model_;
    delete model_;
}
void ValkyrieRobotModel::UpdateModel(const Vector & q, const Vector & qdot){
    UpdateKinematicsCustom(*model_, &q, &qdot, NULL);
    dyn_model_->UpdateDynamics(q, qdot);
    kin_model_->UpdateKinematics(q, qdot);
}


void ValkyrieRobotModel::UpdateModel(int timestep, const sejong::Vector & q, const sejong::Vector & qdot){
  if(timestep != last_timestep_model_update){
    UpdateModel(q, qdot);
    last_timestep_model_update = timestep;
  }
}

void ValkyrieRobotModel::getCentroidInertia(sejong::Matrix & Icent){
    kin_model_->getCentroidInertia(Icent);
}

void ValkyrieRobotModel::getCentroidJacobian(sejong::Matrix & Jcent){
  Jcent.setZero();
    kin_model_->getCentroidJacobian(Jcent);
}

void ValkyrieRobotModel::UpdateKinematics(const Vector & q, const Vector &qdot){
    UpdateKinematicsCustom(*model_, &q, &qdot, NULL);
}

bool ValkyrieRobotModel::getInverseMassInertia(sejong::Matrix & Ainv) {
    return dyn_model_->getInverseMassInertia(Ainv);
}

bool ValkyrieRobotModel::getMassInertia(sejong::Matrix & A) {
    return dyn_model_->getMassInertia(A);
}

bool ValkyrieRobotModel::getGravity(Vector & grav) {
    return dyn_model_->getGravity(grav);
}

bool ValkyrieRobotModel::getCoriolis(Vector & coriolis) {
    return dyn_model_->getCoriolis(coriolis);
}

void ValkyrieRobotModel::getFullJacobian(const Vector & q, int link_id, sejong::Matrix & J) const {
  J.setZero();
  kin_model_->getJacobian(q, link_id, J);
}
void ValkyrieRobotModel::getFullJacobianDot(const Vector & q, const Vector & qdot, int link_id, sejong::Matrix & Jdot) const {
  kin_model_->getJacobianDot6D_Analytic(q, qdot, link_id, Jdot);
}

void ValkyrieRobotModel::getPosition(const Vector & q,
                             int link_id, Vect3 & pos) {
    kin_model_->getPosition(q, link_id, pos);
}
void ValkyrieRobotModel::getOrientation(const Vector & q,
                               int link_id, sejong::Quaternion & ori) {
    kin_model_->getOrientation(q, link_id, ori);
}
void ValkyrieRobotModel::getVelocity(const Vector & q, const Vector &qdot,
                            int link_id, Vect3 & vel) {
    kin_model_->getVelocity(q, qdot, link_id, vel);
}
void ValkyrieRobotModel::getAngVel(const Vector & q, const Vector & qdot,
                          int link_id, Vect3 & ang_vel){
    kin_model_->getAngVel(q, qdot, link_id, ang_vel);
}

void ValkyrieRobotModel::getCoMJacobian(const Vector & q, sejong::Matrix & J){
    J = sejong::Matrix::Zero(3, model_->qdot_size);
    kin_model_->getCoMJacobian(q, J);
}

void ValkyrieRobotModel::getCoMPosition(const Vector & q, Vect3 & com_pos, bool update){
  com_pos = kin_model_->com_pos_;
    // kin_model_->getCoMPos(q, com_pos, update);
}

void ValkyrieRobotModel::getCoMVelocity(const Vector & q, const Vector & qdot, Vect3 & com_vel){
    kin_model_->getCoMVel(q, qdot, com_vel);
}
void ValkyrieRobotModel::getCentroidVelocity(sejong::Vector & centroid_vel){
  centroid_vel = kin_model_->centroid_vel_;
  // kin_model_->getCentroidVelocity(centroid_vel);
}
