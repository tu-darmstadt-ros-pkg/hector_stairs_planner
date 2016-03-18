
function[] = genmprim_obelix(outfilename)

%
%generates motion primitives and saves them into file
%
%written by Maxim Likhachev
%---------------------------------------------------
%

%defines

LINESEGMENT_MPRIMS = 1; %set the desired type of motion primitives
UNICYCLE_MPRIMS = 0;



if LINESEGMENT_MPRIMS == 1
    resolution = 0.05;
    numberofangles = 16; %preferably a power of 2, definitely multiple of 8
    numberofprimsperangle = 16;

    %multipliers (multiplier is used as costmult*cost)
    forwardcostmult = 5;
    forwardcostangle_22_5 = 6;
    forwardcostangle_45 = 7;
    forwardcostangle_67_5 = 8;
    forwardcostangle_90 = 9;
    forwardcostangleabove = 10;
    
    %note, what is shown x,y,theta changes (not absolute numbers)
    
    %0 degreees
    basemprimendpts0_c = zeros(numberofprimsperangle, 4); %x,y,theta,costmult 
    %x aligned with the heading of the robot, angles are positive
    %counterclockwise
    basemprimendpts0_c(1,:) = [2 0 0 forwardcostmult];
    basemprimendpts0_c(2,:) = [2 1 1 forwardcostangle_22_5];
    basemprimendpts0_c(3,:) = [2 2 2 forwardcostangle_45];
    basemprimendpts0_c(4,:) = [1 2 3 forwardcostangle_67_5];
    basemprimendpts0_c(5,:) = [0 2 4 forwardcostangle_90];
    basemprimendpts0_c(6,:) = [-1 2 5 forwardcostangleabove];
    basemprimendpts0_c(7,:) = [-2 2 6 forwardcostangleabove];
    basemprimendpts0_c(8,:) = [-2 1 7 forwardcostangleabove];
    basemprimendpts0_c(9,:) = [-2 0 8 forwardcostangleabove];
    basemprimendpts0_c(10,:) = [-2 -1 -7 forwardcostangleabove];
    basemprimendpts0_c(11,:) = [-2 -2 -6 forwardcostangleabove];
    basemprimendpts0_c(12,:) = [-1 -2 -5 forwardcostangleabove];
    basemprimendpts0_c(13,:) = [0 -2 -4 forwardcostangle_90];
    basemprimendpts0_c(14,:) = [1 -2 -3 forwardcostangle_67_5];
    basemprimendpts0_c(15,:) = [2 -2 -2 forwardcostangle_45];
    basemprimendpts0_c(16,:) = [2 -1 -1 forwardcostangle_22_5];
    
    %45 degrees
    basemprimendpts45_c = zeros(numberofprimsperangle, 4); %x,y,theta,costmult (multiplier is used as costmult*cost)
    %x aligned with the heading of the robot, angles are positive
    %counterclockwise
    basemprimendpts0_c(1,:) = [2 0 0 forwardcostmult];
    basemprimendpts0_c(2,:) = [2 1 1 forwardcostangle_22_5];
    basemprimendpts0_c(3,:) = [2 2 2 forwardcostangle_45];
    basemprimendpts0_c(4,:) = [1 2 3 forwardcostangle_67_5];
    basemprimendpts0_c(5,:) = [0 2 4 forwardcostangle_90];
    basemprimendpts0_c(6,:) = [-1 2 5 forwardcostangleabove];
    basemprimendpts0_c(7,:) = [-2 2 6 forwardcostangleabove];
    basemprimendpts0_c(8,:) = [-2 1 7 forwardcostangleabove];
    basemprimendpts0_c(9,:) = [-2 0 8 forwardcostangleabove];
    basemprimendpts0_c(10,:) = [-2 -1 -7 forwardcostangleabove];
    basemprimendpts0_c(11,:) = [-2 -2 -6 forwardcostangleabove];
    basemprimendpts0_c(12,:) = [-1 -2 -5 forwardcostangleabove];
    basemprimendpts0_c(13,:) = [0 -2 -4 forwardcostangle_90];
    basemprimendpts0_c(14,:) = [1 -2 -3 forwardcostangle_67_5];
    basemprimendpts0_c(15,:) = [2 -2 -2 forwardcostangle_45];
    basemprimendpts0_c(16,:) = [2 -1 -1 forwardcostangle_22_5];
    
    %22.5 degrees
    basemprimendpts22p5_c = zeros(numberofprimsperangle, 4); %x,y,theta,costmult (multiplier is used as costmult*cost)
    %x aligned with the heading of the robot, angles are positive
    %counterclockwise
    basemprimendpts0_c(1,:) = [2 0 0 forwardcostmult];
    basemprimendpts0_c(2,:) = [2 1 1 forwardcostangle_22_5];
    basemprimendpts0_c(3,:) = [2 2 2 forwardcostangle_45];
    basemprimendpts0_c(4,:) = [1 2 3 forwardcostangle_67_5];
    basemprimendpts0_c(5,:) = [0 2 4 forwardcostangle_90];
    basemprimendpts0_c(6,:) = [-1 2 5 forwardcostangleabove];
    basemprimendpts0_c(7,:) = [-2 2 6 forwardcostangleabove];
    basemprimendpts0_c(8,:) = [-2 1 7 forwardcostangleabove];
    basemprimendpts0_c(9,:) = [-2 0 8 forwardcostangleabove];
    basemprimendpts0_c(10,:) = [-2 -1 -7 forwardcostangleabove];
    basemprimendpts0_c(11,:) = [-2 -2 -6 forwardcostangleabove];
    basemprimendpts0_c(12,:) = [-1 -2 -5 forwardcostangleabove];
    basemprimendpts0_c(13,:) = [0 -2 -4 forwardcostangle_90];
    basemprimendpts0_c(14,:) = [1 -2 -3 forwardcostangle_67_5];
    basemprimendpts0_c(15,:) = [2 -2 -2 forwardcostangle_45];
    basemprimendpts0_c(16,:) = [2 -1 -1 forwardcostangle_22_5];
    
    %11.25 degrees
    basemprimendpts11p25_c = zeros(numberofprimsperangle, 4); %x,y,theta,costmult (multiplier is used as costmult*cost)
    %x aligned with the heading of the robot, angles are positive
    %counterclockwise
    basemprimendpts0_c(1,:) = [2 0 0 forwardcostmult];
    basemprimendpts0_c(2,:) = [2 1 1 forwardcostangle_22_5];
    basemprimendpts0_c(3,:) = [2 2 2 forwardcostangle_45];
    basemprimendpts0_c(4,:) = [1 2 3 forwardcostangle_67_5];
    basemprimendpts0_c(5,:) = [0 2 4 forwardcostangle_90];
    basemprimendpts0_c(6,:) = [-1 2 5 forwardcostangleabove];
    basemprimendpts0_c(7,:) = [-2 2 6 forwardcostangleabove];
    basemprimendpts0_c(8,:) = [-2 1 7 forwardcostangleabove];
    basemprimendpts0_c(9,:) = [-2 0 8 forwardcostangleabove];
    basemprimendpts0_c(10,:) = [-2 -1 -7 forwardcostangleabove];
    basemprimendpts0_c(11,:) = [-2 -2 -6 forwardcostangleabove];
    basemprimendpts0_c(12,:) = [-1 -2 -5 forwardcostangleabove];
    basemprimendpts0_c(13,:) = [0 -2 -4 forwardcostangle_90];
    basemprimendpts0_c(14,:) = [1 -2 -3 forwardcostangle_67_5];
    basemprimendpts0_c(15,:) = [2 -2 -2 forwardcostangle_45];
    basemprimendpts0_c(16,:) = [2 -1 -1 forwardcostangle_22_5];
    
    %33.75 degrees
    basemprimendpts33p75_c = zeros(numberofprimsperangle, 4); %x,y,theta,costmult 
    %x aligned with the heading of the robot, angles are positive
    %counterclockwise
    basemprimendpts0_c(1,:) = [2 0 0 forwardcostmult];
    basemprimendpts0_c(2,:) = [2 1 1 forwardcostangle_22_5];
    basemprimendpts0_c(3,:) = [2 2 2 forwardcostangle_45];
    basemprimendpts0_c(4,:) = [1 2 3 forwardcostangle_67_5];
    basemprimendpts0_c(5,:) = [0 2 4 forwardcostangle_90];
    basemprimendpts0_c(6,:) = [-1 2 5 forwardcostangleabove];
    basemprimendpts0_c(7,:) = [-2 2 6 forwardcostangleabove];
    basemprimendpts0_c(8,:) = [-2 1 7 forwardcostangleabove];
    basemprimendpts0_c(9,:) = [-2 0 8 forwardcostangleabove];
    basemprimendpts0_c(10,:) = [-2 -1 -7 forwardcostangleabove];
    basemprimendpts0_c(11,:) = [-2 -2 -6 forwardcostangleabove];
    basemprimendpts0_c(12,:) = [-1 -2 -5 forwardcostangleabove];
    basemprimendpts0_c(13,:) = [0 -2 -4 forwardcostangle_90];
    basemprimendpts0_c(14,:) = [1 -2 -3 forwardcostangle_67_5];
    basemprimendpts0_c(15,:) = [2 -2 -2 forwardcostangle_45];
    basemprimendpts0_c(16,:) = [2 -1 -1 forwardcostangle_22_5];
    
    
elseif UNICYCLE_MPRIMS == 1
    fprintf(1, 'ERROR: unsupported mprims type\n');
    return;
else
    fprintf(1, 'ERROR: undefined mprims type\n');
    return;    
end;
    
    
fout = fopen(outfilename, 'w');


%write the header
fprintf(fout, 'resolution_m: %f\n', resolution);
fprintf(fout, 'numberofangles: %d\n', numberofangles);
fprintf(fout, 'totalnumberofprimitives: %d\n', numberofprimsperangle*numberofangles);

%iterate over angles
for angleind = 1:numberofangles
    
    figure(1);
    hold off;

    text(0, 0, int2str(angleind));
    
    %iterate over primitives    
    for primind = 1:numberofprimsperangle
        fprintf(fout, 'primID: %d\n', primind-1);
        fprintf(fout, 'startangle_c: %d\n', angleind-1);

        %current angle
        currentangle = (angleind-1)*2*pi/numberofangles;
        currentangle_36000int = round((angleind-1)*36000/numberofangles);
        
        %compute which template to use
        if (rem(currentangle_36000int, 9000) == 0)
            basemprimendpts_c = basemprimendpts0_c(primind,:);    
            angle = currentangle;
        elseif (rem(currentangle_36000int, 4500) == 0)
            basemprimendpts_c = basemprimendpts45_c(primind,:);
            angle = currentangle - 45*pi/180;
        elseif (rem(currentangle_36000int-7875, 9000) == 0)
            basemprimendpts_c = basemprimendpts33p75_c(primind,:);
            basemprimendpts_c(1) = basemprimendpts33p75_c(primind, 2); %reverse x and y
            basemprimendpts_c(2) = basemprimendpts33p75_c(primind, 1);
            basemprimendpts_c(3) = -basemprimendpts33p75_c(primind, 3); %reverse the angle as well
            angle = currentangle - 78.75*pi/180;
            fprintf(1, '78p75\n');
        elseif (rem(currentangle_36000int-6750, 9000) == 0)
            basemprimendpts_c = basemprimendpts22p5_c(primind,:);
            basemprimendpts_c(1) = basemprimendpts22p5_c(primind, 2); %reverse x and y
            basemprimendpts_c(2) = basemprimendpts22p5_c(primind, 1);
            basemprimendpts_c(3) = -basemprimendpts22p5_c(primind, 3); %reverse the angle as well
            %fprintf(1, '%d %d %d onto %d %d %d\n', basemprimendpts22p5_c(1), basemprimendpts22p5_c(2), basemprimendpts22p5_c(3), ...
            %    basemprimendpts_c(1), basemprimendpts_c(2), basemprimendpts_c(3));
            angle = currentangle - 67.5*pi/180;
            fprintf(1, '67p5\n');            
        elseif (rem(currentangle_36000int-5625, 9000) == 0)
            basemprimendpts_c = basemprimendpts11p25_c(primind,:);
            basemprimendpts_c(1) = basemprimendpts11p25_c(primind, 2); %reverse x and y
            basemprimendpts_c(2) = basemprimendpts11p25_c(primind, 1);
            basemprimendpts_c(3) = -basemprimendpts11p25_c(primind, 3); %reverse the angle as well
            angle = currentangle - 56.25*pi/180;
            fprintf(1, '56p25\n');
        elseif (rem(currentangle_36000int-3375, 9000) == 0)
            basemprimendpts_c = basemprimendpts33p75_c(primind,:);
            angle = currentangle - 33.75*pi/180;
            fprintf(1, '33p75\n');
        elseif (rem(currentangle_36000int-2250, 9000) == 0)
            basemprimendpts_c = basemprimendpts22p5_c(primind,:);
            angle = currentangle - 22.5*pi/180;
            fprintf(1, '22p5\n');
        elseif (rem(currentangle_36000int-1125, 9000) == 0)
            basemprimendpts_c = basemprimendpts11p25_c(primind,:);
            angle = currentangle - 11.25*pi/180;
            fprintf(1, '11p25\n');
        else
            fprintf(1, 'ERROR: invalid angular resolution. angle = %d\n', currentangle_36000int);
            return;
        end;
        
        %now figure out what action will be        
        baseendpose_c = basemprimendpts_c(1:3);
        additionalactioncostmult = basemprimendpts_c(4);        
        endx_c = round(baseendpose_c(1)*cos(angle) - baseendpose_c(2)*sin(angle));        
        endy_c = round(baseendpose_c(1)*sin(angle) + baseendpose_c(2)*cos(angle));
        endtheta_c = rem(angleind - 1 + baseendpose_c(3), numberofangles);
        endpose_c = [endx_c endy_c endtheta_c];
        
        fprintf(1, 'rotation angle=%f\n', angle*180/pi);
        
        if baseendpose_c(2) == 0 & baseendpose_c(3) == 0
            %fprintf(1, 'endpose=%d %d %d\n', endpose_c(1), endpose_c(2), endpose_c(3));
        end;
        
        %generate intermediate poses (remember they are w.r.t 0,0 (and not
        %centers of the cells)
        numofsamples = 3;
        intermcells_m = zeros(numofsamples,3);
        if LINESEGMENT_MPRIMS == 1
            startpt = [0 0 currentangle];
            endpt = [endpose_c(1)*resolution endpose_c(2)*resolution ...
                rem(angleind - 1 + baseendpose_c(3), numberofangles)*2*pi/numberofangles];
            intermcells_m = zeros(numofsamples,3);
            for iind = 1:numofsamples
                intermcells_m(iind,:) = [startpt(1) + (endpt(1) - startpt(1))*(iind-1)/(numofsamples-1) ...
                                        startpt(2) + (endpt(2) - startpt(2))*(iind-1)/(numofsamples-1) ...
                                        0];
                rotation_angle = (baseendpose_c(3) ) * (2*pi/numberofangles);
                intermcells_m(iind,3) = rem(startpt(3) + (rotation_angle)*(iind-1)/(numofsamples-1), 2*pi);
            end;
        end;
    
        %write out
        fprintf(fout, 'endpose_c: %d %d %d\n', endpose_c(1), endpose_c(2), endpose_c(3));
        fprintf(fout, 'additionalactioncostmult: %d\n', additionalactioncostmult);
        fprintf(fout, 'intermediateposes: %d\n', size(intermcells_m,1));
        for interind = 1:size(intermcells_m, 1)
            fprintf(fout, '%.4f %.4f %.4f\n', intermcells_m(interind,1), intermcells_m(interind,2), intermcells_m(interind,3));
        end;
        
        plot(intermcells_m(:,1), intermcells_m(:,2));
        text(intermcells_m(numofsamples,1), intermcells_m(numofsamples,2), int2str(endpose_c(3)));
        hold on;
        
    end;
    grid;
    pause;
end;
        
fclose('all');
