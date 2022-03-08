use futures::stream::{Stream, StreamExt};
use k::nalgebra::Quaternion;
use k::prelude::InverseKinematicsSolver;
use k::{Chain, Node};
use k::{Isometry3, Translation3, UnitQuaternion, Vector3};
use r2r::geometry_msgs::msg::TransformStamped;
use r2r::sensor_msgs::msg::JointState;
use r2r::std_msgs::msg::Header;
use r2r::std_srvs::srv::Trigger;
use r2r::tf_tools_msgs::srv::LookupTransform;
use r2r::{ParameterValue, ServiceRequest};
use r2r::QosProfile;
use std::fs::File;
use std::io::Write;
use std::sync::{Arc, Mutex};
use tempfile::tempdir;

pub static NODE_ID: &'static str = "teaching_ghost";
pub static SIM_RATE_MS: u64 = 10;

#[derive(Default)]
pub struct Parameters {
    pub acceleration: f64,
    pub velocity: f64,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // setup the node
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, NODE_ID, "")?;

    // handle parameters passed on from the launch filessimple_robot_simulator
    let params = node.params.clone();
    let params_things = params.lock().unwrap(); // OK to panic
    let urdf_raw = params_things.get("urdf_raw");
    let initial_joint_state = params_things.get("initial_joint_state");
    let initial_face_plate_id_param = params_things.get("initial_face_plate_id");
    let initial_tcp_id_param = params_things.get("initial_tcp_id");

    let initial_face_plate_id = match initial_face_plate_id_param {
        Some(p) => match p {
            ParameterValue::String(value) => value.to_string(),
            _ => {
                r2r::log_warn!(
                    NODE_ID,
                    "Parameter 'initial_face_plate_id' has to be of type String."
                );
                "unknown".to_string()
            }
        },
        None => {
            r2r::log_warn!(NODE_ID, "Parameter 'initial_face_plate_id' not specified.");
            "unknown".to_string()
        }
    };

    let initial_tcp_id = match initial_tcp_id_param {
        Some(p) => match p {
            ParameterValue::String(value) => value.to_string(),
            _ => {
                r2r::log_warn!(
                    NODE_ID,
                    "Parameter 'initial_tcp_id' has to be of type String."
                );
                "unknown".to_string()
            }
        },
        None => {
            r2r::log_warn!(NODE_ID, "Parameter 'initial_tcp_id' not specified.");
            "unknown".to_string()
        }
    };

    // make a manipulatable kinematic chain using a urdf or through the xacro pipeline
    let (chain, joints, links) = match urdf_raw {
        Some(p2) => match p2 {
            ParameterValue::String(urdf) => chain_from_urdf_raw(urdf).await,
            _ => {
                r2r::log_error!(NODE_ID, "Parameter 'urdf_raw' has to be of type String.");
                panic!() // OK to panic, makes no sense to continue without a urdf
            }
        },
        None => {
            r2r::log_error!(NODE_ID, "Parameter 'urdf_raw' not specified.");
            panic!() // OK to panic, makes no sense to continue without a urdf
        }
    };

    // did we get what we expected
    r2r::log_info!(NODE_ID, "Found joints: {:?}", joints);
    r2r::log_info!(NODE_ID, "Found links: {:?}", links);

    let initial_joint_value = JointState {
        header: Header {
            ..Default::default()
        },
        name: joints.clone(),
        position: match initial_joint_state {
            Some(p) => match p {
                ParameterValue::StringArray(joints) => joints
                    .iter()
                    .map(|j| j.parse::<f64>().unwrap_or_default())
                    .collect(),
                _ => {
                    r2r::log_warn!(
                        NODE_ID,
                        "Parameter 'initial_joint_state' has to be of type StringArray."
                    );
                    chain
                    .iter_joints()
                    .map(|_| 0.0)
                    .collect()
                }
            },
            None => {
                r2r::log_info!(
                    NODE_ID,
                    "Parameter 'initial_joint_state' not specified, avg of limits will be used."
                );
                chain
                    .iter_joints()
                    .map(|x| match x.limits {
                        Some(l) => (l.max + l.min) / 2.0,
                        None => 0.0,
                    })
                    .collect()
            }
        },
        ..Default::default()
    };

    // initial joint value clone for the reset service
    let initial_joint_value_clone = initial_joint_value.clone();

    // where is the ghost now
    let joint_state = Arc::new(Mutex::new(initial_joint_value));

    // the current kinematic chain of the ghost robot
    let chain = Arc::new(Mutex::new(chain.clone()));

    // so that the ghosts knows how to calculate inverse kinematics
    let current_face_plate = Arc::new(Mutex::new(initial_face_plate_id));

    // so that the ghosts knows how to calculate inverse kinematics
    let current_tcp = Arc::new(Mutex::new(initial_tcp_id));

    // a service to reset the ghost if it gets stuck
    let reset_ghost_service = node.create_service::<Trigger::Service>("reset_ghost")?;

    // listen to the teaching marker pose to calculate inverse kinematics from
    let teaching_pose_subscriber =
        node.subscribe::<TransformStamped>("teaching_pose", QosProfile::default())?;

    // publish the ghost joint state of the robot at the simulation rate
    let pub_timer = node.create_wall_timer(std::time::Duration::from_millis(SIM_RATE_MS))?;
    let ghost_state_publisher =
        node.create_publisher::<JointState>("ghost/joint_states", QosProfile::default())?;

    // spawn a tokio task to handle publishing the ghost's joint state
    let joint_state_clone_1 = joint_state.clone();
    tokio::task::spawn(async move {
        match ghost_publisher_callback(ghost_state_publisher, pub_timer, &joint_state_clone_1).await
        {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Joint state publisher failed with: '{}'.", e),
        };
    });

    // spawn a tokio task to listen to incomming teaching marker poses
    let joint_state_clone_2 = joint_state.clone();
    let current_chain_clone_1 = chain.clone();
    let current_face_plate_clone_1 = current_face_plate.clone();
    let current_tcp_clone_1 = current_tcp.clone();
    tokio::task::spawn(async move {
        match teaching_subscriber_callback(
            teaching_pose_subscriber,
            &current_chain_clone_1,
            &current_face_plate_clone_1,
            &current_tcp_clone_1,
            &joint_state_clone_2,
        )
        .await
        {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Teaching mode subscriber failed with {}.", e),
        };
    });

    // a client that asks a tf lookup service for transformations between frames in the tf tree
    let tf_lookup_client = node.create_client::<LookupTransform::Service>("tf_lookup")?;
    let waiting_for_tf_lookup_server = node.is_available(&tf_lookup_client)?;

    // keep the node alive
    let handle = std::thread::spawn(move || loop {
        node.spin_once(std::time::Duration::from_millis(100));
    });

    // before the other things in this node can start, it makes sense to wait for the tf lookup service to become alive
    r2r::log_warn!(NODE_ID, "Waiting for tf Lookup service...");
    waiting_for_tf_lookup_server.await?;
    r2r::log_info!(NODE_ID, "tf Lookup Service available.");

    // // offer a service to enable or disable remote control
    let joint_state_clone_3 = joint_state.clone();
    tokio::task::spawn(async move {
        let result = reset_ghost_server(reset_ghost_service, &joint_state_clone_3, &initial_joint_value_clone).await;
        match result {
            Ok(()) => r2r::log_info!(NODE_ID, "Remote Control Service call succeeded."),
            Err(e) => r2r::log_error!(NODE_ID, "Remote Control Service call failed with: {}.", e),
        };
    });

    let current_chain_clone_2 = chain.clone();
    let current_face_plate_clone_2 = current_face_plate.clone();
    let current_tcp_clone_2 = current_tcp.clone();
    update_kinematic_chain(
        &tf_lookup_client,
        &current_chain_clone_2,
        &current_face_plate_clone_2,
        &current_tcp_clone_2,
    )
    .await;

    r2r::log_info!(NODE_ID, "Node started.");

    handle.join().unwrap();

    Ok(())
}

// if going throught the launch file - xacro - robot description pipeline,
// a raw urdf is provided, which has to be put in a temp file for the
// kinematic chain generator to access (there might be a nicer way to do this)
async fn chain_from_urdf_raw(urdf: &str) -> (Chain<f64>, Vec<String>, Vec<String>) {
    // create the temp directory to store the urdf file in
    let dir = match tempdir() {
        Ok(d) => d,
        Err(e) => {
            r2r::log_error!(
                NODE_ID,
                "Failed to generate temporary urdf directory with: '{}'.",
                e
            );
            panic!() // OK to panic, makes no sense to continue without a urdf.
        }
    };

    // create the temporary urdf file
    let urdf_path = dir.path().join("temp_urdf.urdf");
    let mut file = match File::create(urdf_path.clone()) {
        Ok(f) => {
            r2r::log_info!(NODE_ID, "Generated temporary urdf file at: {:?}", urdf_path);
            f
        }
        Err(e) => {
            r2r::log_error!(
                NODE_ID,
                "Failed to generate temporary urdf file with: '{}'.",
                e
            );
            panic!() // OK to panic, makes no sense to continue without a urdf.
        }
    };

    // dump the raw urdf to the generated file
    match write!(file, "{}", urdf) {
        Ok(()) => (),
        Err(e) => {
            r2r::log_error!(
                NODE_ID,
                "Failed to write to the temporary urdf file with: '{}'.",
                e
            );
            panic!() // OK to panic, makes no sense to continue without a urdf.
        }
    };

    let (c, j, l) = make_chain(match urdf_path.to_str() {
        Some(s) => s,
        None => {
            r2r::log_error!(NODE_ID, "Failed to convert path to string slice.");
            panic!()
        }
    })
    .await;

    drop(file);

    // once we have the chain, we don't need the urdf anymore
    match dir.close() {
        Ok(()) => (),
        Err(e) => {
            r2r::log_error!(
                NODE_ID,
                "Failed to close and remove the temporary urdf directory with: '{}'.",
                e
            );
        }
    };

    (c, j, l)
}

// actually make the kinematic chain from the urdf file (supplied or generated)
async fn make_chain(urdf_path: &str) -> (Chain<f64>, Vec<String>, Vec<String>) {
    match k::Chain::<f64>::from_urdf_file(urdf_path.clone()) {
        Ok(c) => {
            r2r::log_info!(NODE_ID, "Loading urdf file: '{:?}'.", urdf_path);
            (
                c.clone(),
                c.iter_joints()
                    .map(|j| j.name.clone())
                    .collect::<Vec<String>>(),
                c.iter_links()
                    .map(|l| l.name.clone())
                    .collect::<Vec<String>>(),
            )
        }
        Err(e) => {
            r2r::log_error!(NODE_ID, "Failed to handle urdf with: '{}'.", e);
            panic!() // Still OK to panic, makes no sense to continue without a urdf.
        }
    }
}

// the urdf only holds the joints and links of the robot that are always
// defined in a never-changing way. Sometimes, when the robot is expected
// to always use only one end effector and never change it, it could be reasonable
// to add a new 'fixed' joint and the end effector link to the urdf. In our
// use cases though, we would like to sometimes change tools, which changes the
// tool center point and thus the relationships to the face plate frame. Thus we
// always want to generate a new chain with the current configuration that we
// looked up from the tf. Also, an item's frame that is currently being held
// is also a reasonable tcp to be used when moving somewhere to leave the item.
async fn update_kinematic_chain(
    tf_lookup_client: &r2r::Client<LookupTransform::Service>,
    chain: &Arc<Mutex<Chain<f64>>>,
    face_plate_id: &Arc<Mutex<String>>,
    tcp_id: &Arc<Mutex<String>>,
) -> Option<Chain<f64>> {
    let face_plate_id_local = &face_plate_id.lock().unwrap();
    let tcp_id_local = &tcp_id.lock().unwrap();

    let tcp_in_face_plate =
        lookup_tf(face_plate_id_local, &tcp_id_local, 3000, tf_lookup_client).await;

    match tcp_in_face_plate {
        Some(frame) => {
            // make the new face_plate to tcp joint
            let face_plate_to_tcp_joint: Node<f64> = k::NodeBuilder::<f64>::new()
                .name(&format!("{}-{}", face_plate_id_local, tcp_id_local))
                .translation(Translation3::new(
                    frame.transform.translation.x as f64,
                    frame.transform.translation.y as f64,
                    frame.transform.translation.z as f64,
                ))
                .rotation(UnitQuaternion::from_quaternion(Quaternion::new(
                    frame.transform.rotation.w as f64,
                    frame.transform.rotation.x as f64,
                    frame.transform.rotation.y as f64,
                    frame.transform.rotation.z as f64,
                )))
                // have to make a rot joint, a fixed one is not recognized in DoF
                .joint_type(k::JointType::Rotational {
                    axis: Vector3::y_axis(),
                })
                .finalize()
                .into();

            // specify the tcp link
            let tcp_link = k::link::LinkBuilder::new().name(tcp_id_local).finalize();
            face_plate_to_tcp_joint.set_link(Some(tcp_link));

            let old_chain = chain.lock().unwrap().clone();

            // get the last joint in the chain and hope to get the right one xD
            match old_chain
                .iter_joints()
                .map(|j| j.name.clone())
                .collect::<Vec<String>>()
                .last()
            {
                // fetch the node that is specified by the last joint
                Some(parent) => match old_chain.find(parent) {
                    Some(parent_node) => {
                        // specify the parent of the newly made face_plate-tcp joint
                        face_plate_to_tcp_joint.set_parent(parent_node);

                        // get all the nodes in the chain
                        let mut new_chain_nodes: Vec<k::Node<f64>> =
                            old_chain.iter().map(|x| x.clone()).collect();

                        // add the new joint and generate the new chain
                        new_chain_nodes.push(face_plate_to_tcp_joint);
                        let new_chain = Chain::from_nodes(new_chain_nodes);
                        *chain.lock().unwrap() = new_chain.clone();
                        Some(new_chain)
                    }
                    None => {
                        r2r::log_error!(NODE_ID, "Failed to set parent node.");
                        None
                    }
                },
                None => {
                    r2r::log_error!(NODE_ID, "Failed to find parent node in the chain.");
                    None
                }
            }
        }
        None => None,
    }
}

// get a joint position for the frame to go to
async fn calculate_inverse_kinematics(
    new_chain: &Chain<f64>,
    face_plate_id: &str,
    tcp_id: &str,
    target_frame: &TransformStamped,
    act_joint_state: &JointState,
) -> Option<Vec<f64>> {
    match new_chain.find(&format!("{}-{}", face_plate_id, tcp_id)) {
        Some(ee_joint) => {
            // a chain can have branches, but a serial chain can't
            // so we use that instead to help the solver
            let arm = k::SerialChain::from_end(ee_joint);

            // since we have added a new joint, it is now a n + 1 DoF robot
            let mut positions = act_joint_state.position.clone(); //.lock().unwrap().clone().position;
            positions.push(0.0);

            // the solver needs an initial joint position to be set
            match arm.set_joint_positions(&positions) {
                Ok(()) => {
                    // will have to experiment with these solver parameters
                    // let solver = k::JacobianIkSolver::new(0.01, 0.01, 0.5, 50);
                    let solver = k::JacobianIkSolver::default();

                    let target = Isometry3::from_parts(
                        Translation3::new(
                            target_frame.transform.translation.x as f64,
                            target_frame.transform.translation.y as f64,
                            target_frame.transform.translation.z as f64,
                        ),
                        UnitQuaternion::from_quaternion(Quaternion::new(
                            target_frame.transform.rotation.w as f64,
                            target_frame.transform.rotation.x as f64,
                            target_frame.transform.rotation.y as f64,
                            target_frame.transform.rotation.z as f64,
                        )),
                    );

                    // the last joint has to be rot type to be recognize, but we don't want it to roatate
                    let constraints = k::Constraints {
                        ignored_joint_names: vec![format!("{}-{}", face_plate_id, tcp_id)],
                        ..Default::default()
                    };

                    // solve, but with locking the last joint that we added
                    match solver.solve_with_constraints(&arm, &target, &constraints) {
                        Ok(()) => {
                            // get the solution and remove the (n + 1) - th '0.0' joint value
                            let mut j = arm.joint_positions();
                            match j.pop() {
                                Some(_) => Some(j),
                                None => {
                                    r2r::log_error!(
                                        NODE_ID,
                                        "Failed to shring joint dof to original size.",
                                    );
                                    None
                                }
                            }
                        }
                        Err(e) => {
                            r2r::log_error!(
                                NODE_ID,
                                "Failed to solve with constraints with: '{}'.",
                                e
                            );
                            None
                        }
                    }
                }
                Err(e) => {
                    r2r::log_error!(
                        NODE_ID,
                        "Failed to set joint positions for arm with: '{}'.",
                        e
                    );
                    None
                }
            }
        }
        None => None,
    }
}

//publish the ghost joint state
async fn ghost_publisher_callback(
    publisher: r2r::Publisher<JointState>,
    mut timer: r2r::Timer,
    joint_state: &Arc<Mutex<JointState>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let position = joint_state.lock().unwrap().clone().position;
        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        let now = clock.get_now().unwrap();
        let time_stamp = r2r::Clock::to_builtin_time(&now);

        let updated_joint_state = JointState {
            header: Header {
                stamp: time_stamp.clone(),
                ..Default::default()
            },
            name: joint_state.lock().unwrap().clone().name,
            position,
            ..Default::default()
        };

        match publisher.publish(&updated_joint_state) {
            Ok(()) => (),
            Err(e) => {
                r2r::log_error!(NODE_ID, "Publisher failed to send a message with: '{}'", e);
            }
        };
        timer.tick().await?;
    }
}

// listen to the pose of the teaching marker so that the ghost knows where to go
// the ghost's chain will change whenever the actual chain changes
async fn teaching_subscriber_callback(
    mut subscriber: impl Stream<Item = TransformStamped> + Unpin,
    current_chain: &Arc<Mutex<Chain<f64>>>,
    current_face_plate_id: &Arc<Mutex<String>>,
    current_tcp_id: &Arc<Mutex<String>>,
    joint_state: &Arc<Mutex<JointState>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match subscriber.next().await {
            Some(msg) => {
                let mut new_joint_state = joint_state.lock().unwrap().clone();
                let current_chain_local = current_chain.lock().unwrap().clone();
                let current_face_plate_id_local = current_face_plate_id.lock().unwrap().clone();
                let current_tcp_id_local = current_tcp_id.lock().unwrap().clone();
                let joint_state_local = joint_state.lock().unwrap().clone();
                match (current_face_plate_id_local != "unknown")
                    & (current_tcp_id_local != "unknown")
                {
                    true => {
                        match calculate_inverse_kinematics(
                            &current_chain_local,
                            &current_face_plate_id_local,
                            &current_tcp_id_local,
                            &msg,
                            &joint_state_local,
                        )
                        .await
                        {
                            Some(joints) => {
                                new_joint_state.position = joints;
                                *joint_state.lock().unwrap() = new_joint_state;
                            }
                            None => r2r::log_error!(NODE_ID, "Calculating inverse kinematics failed."),
                        };
                    }
                    false => r2r::log_error!(
                        NODE_ID,
                        "What is unknown?: {:?}, {:?}",
                        current_face_plate_id_local,
                        current_tcp_id_local
                    ),
                }
            }
            None => {
                r2r::log_error!(NODE_ID, "Subscriber did not get the message?");
            }
        }
    }
}

// provide a service to reset the joint position of the ghost to the initial state
async fn reset_ghost_server(
    mut service: impl Stream<Item = ServiceRequest<Trigger::Service>> + Unpin,
    joint_state: &Arc<Mutex<JointState>>,
    initial_joint_state: &JointState
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match service.next().await {
            Some(_) => {
                r2r::log_info!(NODE_ID, "Got reset ghost request.");
                *joint_state.lock().unwrap() = initial_joint_state.clone();
                continue;
            }
            None => (),
        }
    }
}

// ask the lookup service for transforms from its buffer
async fn lookup_tf(
    parent_id: &str,
    child_id: &str,
    deadline: i32,
    tf_lookup_client: &r2r::Client<LookupTransform::Service>,
) -> Option<TransformStamped> {
    let request = LookupTransform::Request {
        parent_id: parent_id.to_string(),
        child_id: child_id.to_string(),
        deadline,
    };

    let response = tf_lookup_client
        .request(&request)
        .expect("Could not send tf Lookup request.")
        .await
        .expect("Cancelled.");

    r2r::log_info!(
        NODE_ID,
        "Request to lookup parent '{}' to child '{}' sent.",
        parent_id,
        child_id
    );

    match response.success {
        true => Some(response.transform),
        false => {
            r2r::log_error!(
                NODE_ID,
                "Couldn't lookup tf for parent '{}' and child '{}'.",
                parent_id,
                child_id
            );
            None
        }
    }
}
