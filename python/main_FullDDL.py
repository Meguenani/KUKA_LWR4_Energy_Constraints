###############################   Controller & Dynamic model    ###############################
# dsimi.rtt : convert RTT::TaskContext * into a python object /
control = xdefw.rtt.Task(ddeployer.load("control", "XDE_SimpleController", "XDESimpleController-gnulinux", "", libdir="/home/anis/ros_workspace/kuka_controller/lib/orocos/gnulinux"))

kukaModel = xde.desc.physic.physic_pb2.MultiBodyModel()
kukaModel.kinematic_tree.CopyFrom(kukaWorld.scene.physical_scene.nodes[0])
kukaModel.meshes.extend(kukaWorld.library.meshes)
kukaModel.mechanism.CopyFrom(kukaWorld.scene.physical_scene.mechanisms[0])
kukaModel.composites.extend(kukaWorld.scene.physical_scene.collision_scene.meshes)
model = physicshelper.createDynamicModel(kukaModel)
#model.setJointPositions(lgsm.vectord(-0.5, -0.3, -0.4, -0.4, -0.2, -0.2, -0.2))
#model.setJointPositions(lgsm.vectord(0.05, 0.68, 0.68, 0.68, 0.68, 0.68, 0.68))
model.setJointPositions(lgsm.vectord(3.5, 0.68, -0.5, 0.68, 0.68, 0.68, 0.68))

kuka = wm.phy.s.GVM.Robot("kuka")

#kuka.setJointPositions(lgsm.vectord(-0.5, -0.3, -0.4, -0.4, -0.2, -0.2, -0.2))
#kuka.setJointPositions(lgsm.vectord(0.05, 0.68, 0.68, 0.68, 0.68, 0.68, 0.68))
kuka.setJointPositions(lgsm.vectord(3.5, 0.68, -0.5, 0.68, 0.68, 0.68, 0.68))
#model.setFreeFlyerPosition(lgsm.Displacement(0.5, 0.5, 0.5, 1, 0, 0, 0))
control.s.setDynModel(str(model.this.__long__()))
###############################################################################################
