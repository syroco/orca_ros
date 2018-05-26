RosPIDControllerProxy::RosPIDControllerProxy(const std::string& robot_name,
                                            const std::string& controller_name,
                                            const std::string& task_name)
: rn_()
, cn_()
, tn_()
{
    nh_ = std::make_shared<ros::NodeHandle>(getNamespacePrefix());

    sc_getSize_
    sc_setProportionalGain_
    sc_getProportionalGain_
    sc_setIntegralGain_
    sc_getIntegralGain_
    sc_setWindupLimit_
    sc_getWindupLimit_
    sc_setDerivativeGain_
    sc_getDerivativeGain_
}

std::string RosPIDControllerProxy::getNamespacePrefix()
{
    return "/orca/"+getRobotName()+"/"+getControllerName()+"/"+getGenericPrefix()+"/"+tn_+"/";
}
