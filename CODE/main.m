function [pose_msg] = project4_mark7()

ip_TurtleBot = '192.168.1.202';    
ip_Matlab = '192.168.1.101';      

setenv('ROS_MASTER_URI', strcat('http://', ip_TurtleBot,':11311'))
setenv('ROS_IP', ip_Matlab)

rosinit(ip_TurtleBot)

if ismember('/raspicam_node/image/compressed', rostopic('list'))
    image_sub = rossubscriber('/raspicam_node/image/compressed');
end

if ismember('/cmd_vel', rostopic('list'))
    velocity_pub = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
end

if ismember('/scan', rostopic('list'))
    laser_sub = rossubscriber('/scan');
end

lane_Output_Video=VideoWriter('Lane_video');
lane_Output_Video.FrameRate= 10;
open(lane_Output_Video);

blob_Output_Video=VideoWriter('Output_video');
blob_Output_Video.FrameRate= 10;
open(blob_Output_Video);

velocity_msg = rosmessage(velocity_pub);

spinVelocity = -0.6;       % Angular velocity (rad/s)
forwardVelocity = 0.1;    % Linear velocity (m/s)

circle_found=0;
while circle_found < 1
    
    img = receive(image_sub);
    img.Format = 'bgr8; jpeg compressed bgr8';
%     imshow(readImage(img))

    img = readImage(img);
    img = imgaussfilt3(img);

    LMin = 50;
    LMax = 100;
    aMin = -128;
    aMax = -10;
    bMin = 0;
    bMax = 128;
    labImg = rgb2lab(img);
    green = (labImg(:,:,1)>=LMin)&(labImg(:,:,1)<=LMax)& ...
            (labImg(:,:,2)>=aMin)&(labImg(:,:,2)<=aMax)& ...
            (labImg(:,:,3)>=bMin)&(labImg(:,:,3)<=bMax);
    
%     imshow(green);
    
    edge_green = edge(green, 'canny');
    
%     imshow(edge_green)
    
    [H,T,R] = hough(edge_green);
    
    P = houghpeaks(H, 5,'threshold',10);
    
    figure(1);
    imshow(img);
    title('lane detection');
    
    try

        lines = houghlines(edge_green, T, R, P, 'FillGap',2500,'MinLength',10);

        hold on
    
        for k = 1:length(lines)
            xy = [lines(k).point1; lines(k).point2];
            plot(xy(:,1), xy(:, 2), 'LineWidth', 2, 'Color', 'green');
        end
        
        hold off

        writeVideo(lane_Output_Video,getframe);

        velocity_msg.Linear.X = forwardVelocity;
        send(velocity_pub,velocity_msg)
        pause(1)
        velocity_msg.Linear.X = 0;
        send(velocity_pub,velocity_msg)

        [~,~,blobSize, imgBW] = detectCircle(img, 1);
        figure(2);
        imshow(imgBW);
        title("Blob Detection");
        writeVideo(blob_Output_Video,getframe);

        if blobSize > 5
            circle_found = 1;
        else
            circle_found = 0;
        end

    catch

        velocity_msg.Linear.X = forwardVelocity;
        send(velocity_pub,velocity_msg)
        pause(1)
        velocity_msg.Linear.X = 0;
        send(velocity_pub,velocity_msg)
        velocity_msg.Angular.Z = spinVelocity;
        send(velocity_pub,velocity_msg)
        pause(2.6)
        velocity_msg.Angular.Z = 0;
        send(velocity_pub,velocity_msg)

        [~,~,blobSize, imgBW] = detectCircle(img, 1);
        figure(2);
        imshow(imgBW);
        title("Blob Detection");
        writeVideo(blob_Output_Video,getframe);

        if blobSize > 10
            circle_found = 1;
        else
            circle_found = 0;
        end  % if ends

    end % try ends

end % while ends

stop = 0;
while stop < 1 

    img = receive(image_sub);
    img.Format = 'bgr8; jpeg compressed bgr8';
%     imshow(readImage(img))

    img = readImage(img);
    img = imgaussfilt3(img);

    [centerX,~,blobSize, imgBW] = detectCircle(img, 1);
%     real_center = 1.6164e+03;
    figure(2);
    imshow(imgBW);
    title("Blob Detection");
    writeVideo(blob_Output_Video, getframe);

    if blobSize > 5 && blobSize <= 210

        real_center = 320;

        if centerX ~= real_center

            velocity_msg.Angular.Z = 0.001*(real_center - centerX);
            send(velocity_pub,velocity_msg)
            pause(0.5)
            velocity_msg.Angular.Z = 0;
            send(velocity_pub,velocity_msg)

        end
            
        scan_data = receive(laser_sub);
        figure(3);
        plot(scan_data);
        data = readCartesian(scan_data);

        x = data(:,1);
        y = data(:,2);
        y_thold = 0.11;
        x_thold = 0.3;

%         dist = sqrt(x.^2 + y.^2);
%         minDist = min(dist);
%         display(minDist)

        object_det = 0;

        for i = 1:length(x)

            if ((y(i) > - y_thold) && (y(i) <= y_thold)) && ((x(i) > 0) && (x(i) <= x_thold))
            
                object_det = 1;
    
            end % if end
        
        end % for end

        if object_det == 1

            velocity_msg.Angular.Z = spinVelocity;
            send(velocity_pub,velocity_msg)
            pause(2.6)

            velocity_msg.Angular.Z = 0;
            send(velocity_pub,velocity_msg)

            velocity_msg.Linear.X = forwardVelocity;
            send(velocity_pub,velocity_msg)
            pause(1)

            velocity_msg.Linear.X = 0;
            send(velocity_pub,velocity_msg)

        else

            velocity_msg.Linear.X = forwardVelocity;
            send(velocity_pub,velocity_msg)
            pause(1)

            velocity_msg.Linear.X = 0;
            send(velocity_pub,velocity_msg)

        end % if end

    elseif blobSize <= 0 || isempty(blobSize)
    
        velocity_msg.Angular.Z = -spinVelocity;
        send(velocity_pub,velocity_msg)
        pause(0.5)
        velocity_msg.Angular.Z = 0;
        send(velocity_pub,velocity_msg)
    
    else

        stop = 1;
    
    end % if ends

end % while ends

rosshutdown