function Function_library(axes_handle, m, sudu)
   
    gif_filename = 'Motion_trajectory.gif'; 
    if exist(gif_filename, 'file')
        delete(gif_filename); 
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    parentName = 'base';
    robot = rigidBodyTree;
    
    % arm length
    lens = [0.16, 0.16, 0.16, 0.16, 0.16, 0.16, 0.16, 0.16, 0.16, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14];
    
    % Each robotic arm rotation axis
    aaxes = [0 0 1; 1 0 0; 1 0 0; 0 0 1; 1 0 0; 1 0 0; 1 0 0;
                  0 0 1; 1 0 0; 0 0 1; 1 0 0; 1 0 0; 1 0 0; 0 0 1; 0 0 1];
    
    chushi_theta=[-2.2; 0.3; 0.2; 0.2; 0.4; 0.4; 0; 
                  0; 0.2; 0; 0.2; 0; 0; 0; 0];
    
    % Design of a fifteen-segment serial rigid body structure
    for i = 1:15
        bodys_n = ['body' num2str(i)];
        joints_n = ['jnt' num2str(i)];
    
        body = rigidBody(bodys_n);
        % All are rotational joints
        joint = rigidBodyJoint(joints_n, 'revolute');
    
        joint.JointAxis = aaxes(i,:);
        joint.HomePosition = chushi_theta(i);
        setFixedTransform(joint, trvec2tform([0, 0, lens(i)]));
    
        body.Joint = joint;
    
        % Add a visualized stick-like structure
        cyl_radius = 0.034;
        cyl_length = lens(i);
        addVisual(body, 'Cylinder', [cyl_radius, cyl_length]);  
    
        addBody(robot, body, parentName);
        % The current body becomes the parent node of the next one
        parentName = bodys_n; 
    end
    
    End_Effector = rigidBody('endeffector');
    setFixedTransform(End_Effector.Joint, trvec2tform([0, 0, 0.1]));
    addVisual(End_Effector, 'Capsule', [0.04, 0.2]);
    % Connect to the last body
    addBody(robot, End_Effector, parentName); 
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % figure('Color','w');
    % axes_handle = axes;
    % view(3);
    % axis equal;
    % axis([-2, 2, -2, 2, 0, 2.5]); 
    % xlabel('X'); ylabel('Y'); zlabel('Z');
    
    all_axis = zeros(6, 15);
    z_o = 0;
    for i = 1:15
        w = aaxes(i,:)'; 
        q = [0; 0; z_o]; 
        % Linear velocity component
        v = -cross(w, q);    
        all_axis(:, i) = [w; v];
        z_o = z_o + lens(i); 
    end
    
    zero_z = sum(lens) + 0.1;  
    zero_T = [eye(3), [0; 0; zero_z]; 0 0 0 1];
    
    zhongzhi_theta = [0.6; 0.8; 0.2; 0.2; 0; 0.4; 0; 
                  0; 0.2; 0; 0; 0; 0; 0.2; 0];
    
    for i = 1:m
        disp(['Running loop ', num2str(i), '/', num2str(m)]);
    
        % Prevent accumulated error
        curTheta = chushi_theta(:);
        % viztree.Configuration = curTheta;
        % Provide some buffering for visual updates
        pause(0.1);  
        
        % Calculate the current starting and target poses
        theta_T0 = kinematics_forward(all_axis, curTheta, zero_T);
        theta_T_goal = kinematics_forward(all_axis, zhongzhi_theta, zero_T);
    
        curTheta = move_pose(all_axis, curTheta, theta_T_goal, zero_T, robot, axes_handle, gif_filename, sudu);
        curTheta = move_pose(all_axis, curTheta, theta_T0, zero_T, robot, axes_handle, gif_filename,sudu);
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    function twist_vec = To_R6(T_mat)
        twist_vec = [T_mat(3,2);
                     T_mat(1,3);
                     T_mat(2,1);
                     T_mat(1,4);
                     T_mat(2,4);
                     T_mat(3,4)];
    end
    
    
    function show_robot_colored(robot, theta_vec, axes_handle)
        % Convert angle vector
        config = homeConfiguration(robot);
        for i = 1:length(theta_vec)
            config(i).JointPosition = theta_vec(i);
        end
    
        % Plotting
        show(robot, config, 'Parent', axes_handle, ...
             'PreservePlot', true, 'Frames', 'off');
    
        drawnow;
        
        % Represent the rigid body shape
        patches = findall(axes_handle, 'Type', 'Patch');
    
        % Set color
        num_bodies = 15;
        base_colors = [
            0, 0, 0;  
            1, 1, 1;  
        ];
    
        for i = 1:min(num_bodies, length(patches))
            color_idx = mod(i, 2) + 1;  
            patches(i).FaceColor = base_colors(color_idx, :);
        end
    
        % Set the end effector to red
        if ~isempty(patches)
            patches(1).FaceColor = [1, 0, 0]; 
        end
    
    end
    
    
    function s = twistsize(xi)
        w_part = xi(1:3);
        v_part = xi(4:6);
        
        % Check the magnitude of the angular velocity component
        if norm(w_part) == 0
            s = norm(v_part);
        else
            s = norm(w_part);
        end
    end
    
    
    function T_out = kinematics_forward(S, q, M)
        dof = length(q);
        T_out = eye(4);
        
        % Iterate through each joint
        for idx = 1:dof
            T_out = T_out * exp_SC(S(:,idx), q(idx));
        end
    
        T_out = T_out * M;
    end
    
    
    function Ad_T = adjoint(T)
        R_part = T(1:3, 1:3);
        p = T(1:3, 4);
        Ad_T = [ R_part,              zeros(3);
                 R3_T(p) * R_part, R_part ];
    end
    
    
    function endtheta = move_pose(S_list, Thestart, T_goal, zero_T, robot, axes_handle, gif_filename, sudu)
        cur_t = Thestart;
        
        % Forward kinematics
        tt_cur = kinematics_forward(S_list, cur_t, zero_T);
        JJ_cur = jjacobian(S_list, cur_t);
    
        VV_cur_matrix = (T_goal - tt_cur) * inv(tt_cur);
        VV_cur = To_R6(VV_cur_matrix);
    
        step_len = sudu;
        while true
            % Update joint angles
            cur_t = cur_t + step_len * pinv(JJ_cur, 0.05) * VV_cur;
    
            % Update forward kinematics
            tt_cur = kinematics_forward(S_list, cur_t, zero_T);
            JJ_cur = jjacobian(S_list, cur_t);
    
            VV_cur_matrix = (T_goal - tt_cur) * inv(tt_cur);
            VV_cur = To_R6(VV_cur_matrix);
    
            speed = twistsize(VV_cur);
            VV_cur = VV_cur / speed;
    
            % viztree.Configuration = Theta_cur;
            % pause(0.05);
            show_robot_colored(robot, cur_t, axes_handle);
            pause(0.01);
    
            if true
                frame = getframe(axes_handle);        
                im = frame2im(frame);                  
                [A,map] = rgb2ind(im,256);          
                if ~exist(gif_filename, 'file')      
                    imwrite(A,map,gif_filename,'gif','LoopCount',Inf,'DelayTime',0.13);
                else                              
                    imwrite(A,map,gif_filename,'gif','WriteMode','append','DelayTime',0.13);
                end
            end
    
            if speed < 0.045
                break
            end
        end
    
        endtheta = cur_t;
    end
    
    
    function J_space = jjacobian(S, q)
        num = length(q);
        J_space = S(:,1);
        
        % Iterate through each joint
        for idx = 2:num
            T = eye(4);
            for j = 1:(idx - 1)
                T = T * exp_SC(S(:,j), q(j));
            end
    
            J_space = [J_space, adjoint(T) * S(:,idx)];
        end
    end
    
    
    function M = R3_T(vec)
    
        M = [     0      -vec(3)   vec(2);
               vec(3)      0      -vec(1);
              -vec(2)   vec(1)      0   ];
    end
    
    
    function g = exp_SC(axis, angle)
        w = axis(1:3);
        v = axis(4:6);
    
        % Calculate the magnitudes of angular velocity and linear velocity
        w_norm = sqrt(sum(w.^2));
        v_norm = sqrt(sum(v.^2));
    
        if w_norm == 1
            w_hat = R3_T(w);
            G = eye(3)*angle + (1 - cos(angle)) * w_hat + (angle - sin(angle)) * (w_hat * w_hat);
            g = [expm(w_hat * angle), G * v;
                 0 0 0 1];
        end
    
        if w_norm == 0 && v_norm == 1
            g = [eye(3), angle * v;
                 0 0 0 1];
        end
    end
end