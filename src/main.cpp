/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <cmath>
#include <thread>
#include <unordered_map>

#include <iCub/iKin/iKinFwd.h>

#include <yarp/dev/IEncoders.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>


using namespace std::chrono_literals;

using namespace iCub::iKin;

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;


int main(int argc, char** argv)
{
    /* Get the robot name. */

    ResourceFinder rf;
    rf.configure(argc, argv);

    std::string robot_name = rf.check("robot-name", Value("icub")).asString();

    /* Check YARP network. */

    Network network;
    if (!network.checkNetwork())
    {
        std::cout << "YARP network is not available." << std::endl;

        return EXIT_FAILURE;
    }

    /* Configure encoders. */

    std::unordered_map<std::string, PolyDriver> drivers;
    std::unordered_map<std::string, IEncoders*> encoders;

    for (const std::string part_name : {"torso", "head"})
    {
        Property prop;
        prop.put("device", "remote_controlboard");
        prop.put("local", "/fwdkin-inspector/" + part_name);
        prop.put("remote", "/" + robot_name + "/" + part_name);

        if (!drivers[part_name].open(prop))
        {
            std::cout << "Cannot open " + part_name + " driver." << std::endl;

            return EXIT_FAILURE;
        }

        if (!drivers[part_name].view(encoders[part_name]))
        {
            std::cout << "Cannot open encoder interface for " + part_name + "." << std::endl;

            return EXIT_FAILURE;
        }
    }

    /* Configure forward kinematics. */

    iCubHeadCenter head_center("right_v2");
    head_center.releaseLink(0);
    head_center.releaseLink(1);
    head_center.releaseLink(2);

    /* Print the head center transformation. */

    int max_size = 3;
    while (true)
    {
        yarp::sig::Vector chain;

        for (const std::string part_name : {"torso", "head"})
        {
            int size;
            encoders[part_name]->getAxes(&size);

            yarp::sig::Vector part_chain(size);
            encoders[part_name]->getEncoders(part_chain.data());

            int offset = 0;
            if (part_name == "torso")
                offset = -2;

            for (std::size_t i = 0; i < max_size; i++)
                chain.push_back(part_chain(abs(offset + i)));
        }

        Matrix H = head_center.getH(chain * M_PI / 180.0);
        std::cout << "H:" << std::endl;
        std::cout << H.toString() << std::endl << std::endl;

        std::this_thread::sleep_for(30ms);
    }


    return EXIT_SUCCESS;
}
