function val = check_ind(ix,iy,max_val)
val = 1;
if ix<1
    val = 0;
end
if ix>max_val
    val = 0;
end

if iy<1
    val = 0;
end
if iy>max_val
    val = 0;
end