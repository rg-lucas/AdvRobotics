function result_dq = slerp(dq_start, dq_end, alpha)
    if alpha == 0
        result_dq = dq_start;
    elseif alpha == 1
        result_dq = dq_end;
    else
        dot_product = sum(dq_start .* dq_end);
        if dot_product < 0
            dq_start = -dq_start;
            dot_product = -dot_product;
        end

        dot_product = min(1, max(-1, dot_product));
        theta = acos(dot_product);
        result_dq = (sin((1 - alpha) * theta) * dq_start + sin(alpha * theta) * dq_end) / sin(theta);
    end
end
