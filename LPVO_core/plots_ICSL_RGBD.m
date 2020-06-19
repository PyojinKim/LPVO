if (toVisualize)
    %% prerequisite to visualize
    
    p_gc_LPVO = zeros(3, imgIdx);
    for k = 1:imgIdx
        p_gc_LPVO(:,k) = T_gc_LPVO{k}(1:3,4);
    end
    
    %% update RGB image part
    
    axes(ha1); cla;
    plot_segmentation_image(R_cM, sNV, sPP, imageCurForMW, optsLPVO); hold on;
    plot_clustered_line(R_cM, vDV, linePairIdx, lines, optsLPVO);
    plot_tracked_point(ipRelations, cam, optsLPVO);
    plot_display_text(R_cM, sNV, vDV, ipRelations, isTracked, optsLPVO); hold off;
    title('point, line, plane tracking image');
    
    %% update unit sphere part
    
    axes(ha2); cla;
    plot_unit_sphere(1, 18, 0.5); hold on; grid on; axis equal;
    plot_sNV_unit_sphere(1, R_cM, sNV, optsLPVO);
    plot_unit_sphere(0.5, 18, 0.5);
    plot_vDV_unit_sphere(0.5, R_cM, vDV, optsLPVO);
    plot_body_frame(R_cM, '--', 4); hold off;
    view(-1, -71);
    title('unit sphere on SO(3)');
    
    %% update 3D trajectory part
    
    axes(ha3); cla;
    % draw moving trajectory
    plot3(p_gc_LPVO(1,1:imgIdx), p_gc_LPVO(2,1:imgIdx), p_gc_LPVO(3,1:imgIdx), 'm', 'LineWidth', 2); hold on; grid on; axis equal;
    
    % draw camera body and frame
    plot_inertial_frame(0.5);
    RgcLPVO_current = T_gc_LPVO{imgIdx}(1:3,1:3);
    pgcLPVO_current = T_gc_LPVO{imgIdx}(1:3,4);
    plot_camera_frame(RgcLPVO_current, pgcLPVO_current, imageCurForMW, 1.3, 'm'); hold off;
    refresh; pause(0.01);
    
    %% save current figure
    
    if (toSave)
        % save directory for MAT data
        SaveDir = [datasetPath '/ICRA2018'];
        if (~exist( SaveDir, 'dir' ))
            mkdir(SaveDir);
        end
        
        % save directory for images
        SaveImDir = [SaveDir '/LPVO'];
        if (~exist( SaveImDir, 'dir' ))
            mkdir(SaveImDir);
        end
        
        pause(0.01); refresh;
        saveImg = getframe(h);
        imwrite(saveImg.cdata , [SaveImDir sprintf('/%06d.png', imgIdx)]);
    end
end
