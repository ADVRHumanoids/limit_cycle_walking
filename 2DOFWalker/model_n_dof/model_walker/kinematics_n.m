function kinematics = kinematics_n(parent_tree, generalizedVariables, robotData)
% 



    
    if robotData.flagSimulation == 1
        
        q = generalizedVariables;
        z = generalizedVariables(end-1:end);
        dim_qe = size(q,2);
        
    else
        
        qe = generalizedVariables.qe;
        q = qe(1:end-2);
        qe_dot = generalizedVariables.qe_dot;
        z = generalizedVariables.qe(end-1:end);
        z = z.';
        z_dot = generalizedVariables.qe_dot(end-1:end);
        
        q_dot = qe_dot(1:end-2);
        dim_qe = size(qe,2);

    end
    
    if robotData.flagSimulation == 0

        rp_rel = sym(zeros(3,length(parent_tree)));
        rc_rel = sym(zeros(3,length(parent_tree)));
        R_rel = sym(zeros(3,3,length(parent_tree)));
        R_abs = sym(zeros(3,3,length(parent_tree)));
        w_rel = sym(zeros(3,1,length(parent_tree)));
        w_abs = sym(zeros(3,1,length(parent_tree)));
        rp_abs = sym(zeros(3,1,length(parent_tree)));
        rc_abs = sym(zeros(3,1,length(parent_tree)));
        vc_abs = sym(zeros(3,1,length(parent_tree)));
        J = sym(zeros(2, dim_qe));

    else
    %     
        rp_rel = zeros(3,length(parent_tree));
        rc_rel = zeros(3,length(parent_tree));
        R_rel = zeros(3,3,length(parent_tree));
        R_abs = zeros(3,3,length(parent_tree));
        w_rel = zeros(3,1,length(parent_tree));
        w_abs = zeros(3,1,length(parent_tree));
        rp_abs = zeros(3,1,length(parent_tree));
        rc_abs = zeros(3,1,length(parent_tree));
        vc_abs = zeros(3,1,length(parent_tree));
        J = zeros(2, dim_qe);
    %      
    end

        for i = 1:robotData.n_link
            rp_rel(1,i) = robotData.link_length(i);
            rc_rel(1,i) = robotData.com_position(i);
        end


%==========================================================================
    R_rel(:,:,1) = [sin(q(1)) -cos(q(1)) 0;
                    cos(q(1)) sin(q(1))  0;
                    0          0         1];

    for i = 2:length(parent_tree)  

        R_rel(:,:,i) = [ cos(q(i))  sin(q(i))  0;
                        -sin(q(i))  cos(q(i))  0;
                        0           0          1];  
    end

    R_abs(:,:,1) = R_rel(:,:,1); %from parent origin to last child
    for i = 2:length(parent_tree)
        R_abs(:,:,i) = R_rel(:,:,i) * R_abs(:,:,parent_tree(i));
    end

    for i = 1:length(parent_tree)
        rp_abs(:,:,i) = R_abs(:,:,i) * rp_rel(:,i);
    end
    
%     %floating base
%     for i = 1:length(parent_tree)
%         rp_abs(:,:,i) = [z; 0] + rp_abs(:,:,i);
%     end
%==========forward kinematics==============================================
    p(:,:,1) = z + rp_abs(1:2,:,1);
    for i = 2:length(parent_tree)
        p(:,:,i) = p(1:2,:,parent_tree(i)) + rp_abs(1:2,:,i);
    end

%==========================================================================

%==========================================================================
    if robotData.flagSimulation == 0
        
        for i = 1:length(parent_tree)
            rc_abs(:,:,i) = R_abs(:,:,i) * rc_rel(:,i);
        end
        
%         %floating base
%         for i = 1:length(parent_tree)
%             rc_abs(:,:,i) = [z; 0] + rc_abs(:,:,i);
%         end        
%==========forward kinematics CoM==========================================
        pc(:,:,1) = z + rc_abs(1:2,:,1);
        for i = 2:length(parent_tree)
            pc(:,:,i) = z + rp_abs(1:2,:,parent_tree(i)) + rc_abs(1:2,:,i);
        end
%==========================================================================


%======================velocities==========================================

        for i = 1:length(parent_tree)
            w_rel(:,:,i) = [0; 0; q_dot(i)];
        end


        w_abs(:,:,1) = w_rel(:,:,1);
        for i = 2:length(parent_tree)
            w_abs(:,:,i) = w_abs(:,:,(parent_tree(i))) + R_abs(:,:,i) * w_rel(:,:,i);
        end

        %base~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        v_abs(:,:,1) = [z_dot(1); z_dot(2); 0];
        %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        for i = 2:length(parent_tree)
            v_abs(:,:,i) = v_abs(:,:,parent_tree(i)) + cross(w_abs(:,:,i-1), rp_abs(:,:,i-1));
        end


        for i = 1:length(parent_tree)
            vc_abs(:,:,i) = v_abs(:,:,i) + cross(w_abs(:,:,i),rc_abs(:,:,i));
        end
        


%==========jacobian========================================================
for k = 1:length(parent_tree)
    for j = 1:size(p(:,:,k),1)
        for i = 1:dim_qe
            J(j,i,k) = functionalDerivative(p(j,:,k),qe(i));
        end
    end
end
%==========================================================================
        kinematics = struct('parent_tree', parent_tree, ...
                            'rotationMatrix_relative', simplify(R_rel), ...
                            'rotationMatrix_absolute', simplify(R_abs), ...
                            'angularVelocity_relative', simplify(w_rel), ...
                            'angularVelocity_absolute', simplify(w_abs), ...
                            'positionLink_relative', simplify(rp_rel), ...
                            'positionLink_absolute', simplify(rp_abs),...
                            'positionCoM_relative', simplify(rc_rel), ...
                            'positionCoM_absolute', simplify(rc_abs),...
                            'velocityLink_absolute', simplify(v_abs), ...
                            'velocityCoM_absolute', simplify(vc_abs), ...
                            'linksPosition', simplify(p), ...
                            'CoMPosition', simplify(pc), ...
                            'jacobian', simplify(J));
                        
    else
       kinematics = struct('linksPosition', p);


    end

end