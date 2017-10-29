clc
close all
clear all


x = 0:.1:10;
y = 0:.1:10;

x_agent = 1; % start points...
y_agent = 1;

Act = zeros(length(x),length(y));
%V = zeros(length(x),length(y));

[xx,yy] = meshgrid(x,y);
%{\
figure
plot(xx,yy,'k.')
hold on;
%}
x_goal = 9;
y_goal = 9;


x_start = 1;
y_start = 1;

[i_x,i_y]= xy_to_indices(x_goal,y_goal);
ix_goal = i_x;
iy_goal = i_y;

i_do_nth = [i_x,i_y];

Act(i_x,i_y) = 0;
%{\
plot(x_goal,y_goal,'ro')
plot(x_start,y_start,'go')
%}
%function to make objects?

obs_locs = [8.5 1 9 8;
    3 7.5 10 8;
    0 1.5 8 2;
    1 2.5  8.5 3;
    1 3 1.5 9;
    2 4 2.5 10;
    2 4 8 4.5;
    5.5 5 6 9.5;
    3.5 8 4 9.5;
    6.5 8.5 7 10;
    7.5 8 8 9.5
    ];
%consider function
for i = 1:size(obs_locs,1)
    %{\
    patch( [obs_locs(i,1) obs_locs(i,1)  obs_locs(i,3) obs_locs(i,3)  ], ...
        [obs_locs(i,2) obs_locs(i,4)  obs_locs(i,4) obs_locs(i,2)  ],'g' );
    %}
    [ix_obs_st,iy_obs_st]= xy_to_indices( obs_locs(i,1), obs_locs(i,2));
    [ix_obs_en,iy_obs_en]= xy_to_indices( obs_locs(i,3), obs_locs(i,4));
    %{\
    %buffer of .1 (1 grid space)
    x_st = ix_obs_st-1;
    x_en = ix_obs_en+1;
    y_st = iy_obs_st-1;
    y_en = iy_obs_en+1;
    if x_st <= 0
        x_st = ix_obs_st;
    elseif x_en > length(xx)
        x_en = ix_obs_en;
    elseif y_st <= 0
        y_st = iy_obs_st;
    elseif y_en > length(xx)
        y_en = iy_obs_en;
    end
    %}
    Act(x_st:x_en,y_st:y_en) =  200;
    %Act(ix_obs_st:ix_obs_en,iy_obs_st:iy_obs_en) = 200;
end


close all
figure;
V = 20000*ones(size(Act));
filename = ['Value_growth' num2str(i) '.gif'];

changed = 1;
i_V = 1;
%cost to go
while changed == 1
    changed = 0;
    V_old = V;
    for i_x = 1:length(x)
        for i_y = 1:length(y)
            
            if (i_x == ix_goal) &(i_y == iy_goal)
                if V(i_x,i_y) > 0
                    changed = 1;
                    V(i_x,i_y) = 0;
                end
            end
            
            if Act(i_x,i_y) ~= 200;
                iv_x = [1 -1 0 0 1 -1 1  -1];
                iv_y = [0 0 -1 1  1 -1 -1 1];
                V_new = [];
                for i_v = 1:8,
                    val = check_ind(i_x+iv_x(i_v),i_y+iv_y(i_v),length(xx));
                    if val == 1
                        V_new  = V(i_x+iv_x(i_v),i_y+iv_y(i_v)) + 10*sqrt(iv_x(i_v)^2+iv_y(i_v)^2);
                        
                        if V_new< V(i_x,i_y)
                            V(i_x,i_y) = V_new;
                            changed = 1;
                            
                        end
                    end
                end
                
                
            else
                V(i_x,i_y) = 20000;
            end
            
            
            
            
            
        end
    end
    %{\
    surf(yy,xx,V_old);xlabel('X');ylabel('Y');zlabel('Value')
    title(['Value after ' num2str(i_V) ' iterations.'])
    axis([0 10 0 10 -200 25000])
    view(i_V*3,30);
    frame = getframe(1);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if i_V == 1;
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',.1);
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',.1);
    end
    i_V = i_V+1;
    %}
end

%{\
for i = i_V:i_V+60
    surf(yy,xx,V);xlabel('X');ylabel('Y');zlabel('Value')
    title(['Value after ' num2str(i) ' iterations (Converged).'])
    axis([0 10 0 10 -200 25000])
    view(i*3,30);
    frame = getframe(1);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',.1);
    
end



filename = ['Obs_Avoidance' num2str(i) '.gif'];
close all

figure;
plot(xx,yy,'k.')
hold on;
for i = 1:size(obs_locs,1)
    patch( [obs_locs(i,1) obs_locs(i,1)  obs_locs(i,3) obs_locs(i,3)  ], ...
        [obs_locs(i,2) obs_locs(i,4)  obs_locs(i,4) obs_locs(i,2)  ],'g' );
    
end
%}
%%
Va = [];
hold on;
i_move = 1;
x_path=[];
y_path=[];
for i = 1:length(x_agent)
    [i_x,i_y]= xy_to_indices(x_agent(i),y_agent(i));
    stop_mov = 0;
    while stop_mov == 0
        iv_x = [1 -1  0  0  1  -1   1 -1];
        iv_y = [0  0 -1  1  1  -1  -1  1];
        
        for i_v = 1:8,
            Va(i_v) = V(i_x+iv_x(i_v),i_y+iv_y(i_v)) + 10*sqrt(iv_x(i_v)^2+iv_y(i_v)^2);
        end
        
        [V_min , i_vmin]= min(Va);
        x_agent(i) = x(i_x+iv_x(i_vmin));
        y_agent(i) = y(i_y+iv_y(i_vmin));
        x_path = [x_path x_agent(i)];
        y_path = [y_path y_agent(i)];
        %{
        plot(x_agent(i),y_agent(i),'bx')
        plot(x_agent(i),y_agent(i),'b*')
        %}
        
        if (x_agent(i)==x_goal)&(y_agent(i)==y_goal)
            stop_mov = 1;
        end
        
        %{\
        frame = getframe(1);
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
        if i_move == 1;
            imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',.1);
        else
            imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',.1);
        end
        i_move = i_move+1;
        %}
        
        [i_x,i_y]= xy_to_indices(x_agent(i),y_agent(i));
        %pause(0.01);
    end
end

gamma = .1;
beta = .5;

xy_nsmooth=[x_path;y_path];
xy_smooth=xy_nsmooth;
for i=2:length(xy_nsmooth)-1
    xy_smooth(:,i) = xy_smooth(:,i) + beta*(xy_nsmooth(:,i)-xy_smooth(:,i)) + ...
        gamma*(xy_smooth(:,i-1) - 2*xy_smooth(:,i)+xy_smooth(:,i+1));
end
%{\
for i=1:length(xy_smooth)
    plot(xy_smooth(1,i),xy_smooth(2,i),'rx-')
    plot(xy_smooth(1,i),xy_smooth(2,i),'r-*')
    plot(xy_nsmooth(1,i),xy_nsmooth(2,i),'')
    plot(xy_nsmooth(1,i),xy_nsmooth(2,i),'-b*')
    plot(x_start,y_start,'go')
    plot(x_goal,y_goal,'ro')
    frame = getframe(1);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if i_move == 1;
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',.1);
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',.1);
    end
    i_move = i_move+1;
    pause(0.01);
end
T = table([x_path;y_path]',xy_smooth');
T.Properties.VariableNames={'Original_Path' 'Smooth_Path'};
T
%}
%{\
A=[0 1 0 0;
   0 0 0 0;
   0 0 0 1;
   0 0 0 0];
B=[0 1 0 0; 
   0 0 0 1]';
C=[1 0 0 0;
   0 0 1 0];
Ma=rank(ctrb(A,B));
%controller
r=[.1 .1];
q=[1 .005 1 .005];
R=diag(r);
Q=diag(q);
[P,E,K] =care(A,B,Q,R);
%observer
r_k=[.003 .003];
q_k=[1 20 1 20];
R_k=diag(r_k);
Q_k =diag(q_k);
[P_k,E_k,L]=care(A',C',Q_k,R_k);
eig =[eig(A) E E_k]
%}
%{\
%distance
l = 0;
for i=2:length(xy_smooth)
    l = l + sqrt((xy_smooth(1,i) - xy_smooth(1,i-1))^2+(xy_smooth(2,i) - xy_smooth(2,i-1))^2);
end

vel = 1.5; %assuming const speed
total_time = l/vel;
t =linspace(0,total_time,length(x_path));
dt =  t(2) - t(1);
clear y
X(:,1)=[1 0 1 0]';
y(:,1)=C*X;
X_hat(:,1)=[1 0 1 0]';
y_hat(:,1)=C*X_hat;

u(:,1)=-K*X_hat;
for i = 2:length(t)
    u(:,i)= -K*(X_hat(:,i-1) - [xy_smooth(1,i) 0 xy_smooth(2,i) 0]');
    
    X(:,i) = X(:,i-1) + dt*(A*X(:,i-1) + B*u(:,i));
    y(:,i) = C*X(:,i) + sqrt(R_k)*randn(size(C,1),1);

    X_hat(:,i) = X_hat(:,i-1)  +dt * (A*X_hat(:,i-1) + B*u(:,i) +L'*(y(:,i-1)-y_hat(:,i-1)));
    y_hat(:,i) = C*X_hat(:,i) ;
end
figure;
subplot(3,1,1)
plot(t,X(1,:),t,X_hat(1,:),t,X(3,:),t,X_hat(3,:),t,xy_smooth(1,:),t,xy_smooth(2,:))
title_str=['Q = ' num2str(q) ' R = ' num2str(r) ...
    ' Q_k = ' num2str(q_k) ' R_k = ' num2str(r_k)];
title(title_str)
legend('x_{actual}','x_{estimate}','y_{actual}','y_{estimate}','x_{path}','y_{path}','Location','bestoutside')
xlabel('Time')
ylabel('Position')
subplot(3,1,2)
plot(t,X(2,:),t,X_hat(2,:),t,X(4,:),t,X_hat(4,:),t,2*ones(length(t)),t,-2*ones(length(t)))
xlabel('Time')
ylabel('Velocity')
subplot(3,1,3)
plot(t,u(1,:),t,u(2,:),t,ones(length(t)),t,-1*ones(length(t)))
legend('x_{control}','y_{control}')
xlabel('Time')
ylabel('Control')