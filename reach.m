function [new_head, new_tail] = reach(head, tail, target)
   
    % length of the initial line
    line_length = norm(head-tail);
    
    % length of the stretched line
    vector_of_stretched_line_towards_tail = tail - target;
    stretched_line_length = norm(vector_of_stretched_line_towards_tail);
    
    % calculate how much to scale the stretched line
    s = line_length / stretched_line_length;
    
    % new line
    new_head = target;
    new_tail = target + s*vector_of_stretched_line_towards_tail;
    
end