function [h] = getPic(xt,yt,zt,x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4 )
%   GETPIC 此处显示有关此函数的摘要
%   获取图形的数据信息，通过数据信息对图像进行绘画
%  其中，x1表示圆上的点，x2表示第一个关节的末端，x3表示第二个关节的末端，x4表示最末端的位置。
    % 其中返回图形的句柄
    plot3(xt,yt,zt);
    hold on
    xa = [x1,x2];
    ya = [y1,y2];
    za = [z1,z2];
    plot3(xa,ya,za,'-r.','markersize',10,'LineWidth',2)
    
    xb = [x2,x3];
    yb = [y2,y3];
    zb = [z2,z3];
    plot3(xb,yb,zb,'-b.','markersize',10,'LineWidth',2)
    
    xc = [x3,x4];
    yc = [y3,y4];
    zc = [z3,z4];
    plot3(xc,yc,zc)

    plot3(x1,y1,z1,'-g.','markersize',10,'LineWidth',2)
    view(100,30)
    axis([-6 6 -6 6 -6 6])
%     x1=xlabel('X轴');        %x轴标题
%     x2=ylabel('Y轴');        %y轴标题
%     x3=zlabel('Z轴');        %z轴标题
    drawnow;
end

