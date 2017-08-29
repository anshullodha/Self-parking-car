time = solution.phase.time;
states = solution.phase.state;
controls = solution.phase.control;
filename = 'GPOPs_solution.gif';

for i=1:length(states)
    cla;
    plot(states(:,1),states(:,2))
    plot_Environment;
    plotCLMR(states(i,1),states(i,2),states(i,3),states(i,4))
    
    frame = getframe(1);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if i  == 1
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',.01);
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',.01);
    end
    pause(0.01)
    
end
