function [h] = getPic(xt,yt,zt,x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4 )
%   GETPIC �˴���ʾ�йش˺�����ժҪ
%   ��ȡͼ�ε�������Ϣ��ͨ��������Ϣ��ͼ����л滭
%  ���У�x1��ʾԲ�ϵĵ㣬x2��ʾ��һ���ؽڵ�ĩ�ˣ�x3��ʾ�ڶ����ؽڵ�ĩ�ˣ�x4��ʾ��ĩ�˵�λ�á�
    % ���з���ͼ�εľ��
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
%     x1=xlabel('X��');        %x�����
%     x2=ylabel('Y��');        %y�����
%     x3=zlabel('Z��');        %z�����
    drawnow;
end

