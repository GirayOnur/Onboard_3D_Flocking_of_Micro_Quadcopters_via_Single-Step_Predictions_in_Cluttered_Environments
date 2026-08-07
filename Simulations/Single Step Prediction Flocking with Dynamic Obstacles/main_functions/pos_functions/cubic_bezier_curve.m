%generates the points of a cubic Bezier curve defined by four control points:
function [points] = cubic_bezier_curve(P0,P1,P2,P3,num_points)
t_values = linspace(0, 1, num_points); %curve parameter samples
pt_dim = length(P0); %dimension of the control points
points = zeros(length(t_values),pt_dim); %preallocation
for i = 1:length(t_values)
    t = t_values(i);

    %Bernstein basis polynomials of degree 3:
    B0 = (1 - t)^3;
    B1 = 3 * t * (1 - t)^2;
    B2 = 3 * t^2 * (1 - t);
    B3 = t^3;

    point = P0 * B0 + P1 * B1 + P2 * B2 + P3 * B3;
    points(i,:) = reshape(point,1,pt_dim);
end
