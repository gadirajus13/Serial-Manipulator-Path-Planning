% Input: A 2D Segment, S, and a 2D convex polygon Q
% Output: Boolean b, where b is true if interesection
function b = isintersect_linepolygon(S,Q)
    Q = CCW(Q);
    Q = [Q,Q(:,1)];
    p0 = S(:,1);
    p1 = S(:,2);

    % If line segment is single point, check if point is in polygon
    if isequal(p0,p1)
        b = inpolygon(p0(1),p0(2),Q(1,:),Q(2,:));
        return;
    end
    
    tE = 0;
    tL = 1;
    ds = p1 - p0;
    
    for i=1:size(Q,2)-1
        % Calculate edge of Q
        qi = Q(:,i);
        qi1 = Q(:,i+1);
        ei = qi - qi1;

        % Compute the outward normal vector
        e_hat = ei / norm(ei);
        ni = [-e_hat(2); e_hat(1)]; 

        N = -(p0 - qi)'*ni;
        D = ds'*ni;

        if D == 0
            if N < 0
                b = false;
                return;
            end
        end

        t = N/D;
        
        if D < 0
            tE = max(tE,t);
            if tE > tL
                b = false;
                return;
            end
        elseif D > 0
            tL = min(tL,t);
            if tL < tE
                b = false;
                return;
            end
        end

    end
    
    if tE <= tL 
        b = true;
        return;
    else
        b = false;
        return;
    end
end

% Function to get Counter Clockwise Order for Polygon
function ccw_poly = CCW(poly)
    k = length(poly);
    center_x = sum(poly(1,:))/k;
    center_y = sum(poly(2,:))/k;
    for i = 1:k
        dx = poly(1,i)-center_x;
        dy = poly(2,i)- center_y;
        angles(i) = wrapTo2Pi(atan2(dy, dx));
    end

    % Sort by angles to get CCW Order
    [~, order] = sort(angles);
    ccw_poly = poly(:, order);

    % Sort vertices so we start with lowest y value verex in CCW order
    [~, lowest_y_index] = min(ccw_poly(2, :));
    ccw_poly = circshift(ccw_poly, [0, -lowest_y_index + 1]);
end