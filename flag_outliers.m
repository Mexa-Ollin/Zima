function pid = flag_outliers(pid)
for k = 1:numel(pid)
    pid(k).flag = "";
    if ~isfinite(pid(k).T) || pid(k).T <= 0, pid(k).flag = pid(k).flag + " T_bad"; end
    if ~isfinite(pid(k).K),                 pid(k).flag = pid(k).flag + " K_bad"; end
    if isfinite(pid(k).K) && pid(k).K < 0,  pid(k).flag = pid(k).flag + " K_sign"; end
end
end