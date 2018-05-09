
fp1 = fopen('D:\Github\pi_copley_motor_ctl.git\²âÊÔ¼ÇÂ¼\system_test\configuration_one_current_test\motor_log_2018_05_09_1118.txt');

B = textscan(fp1,'%s%s%s%s%s%s%s%s');


ln2 = size(B{1,1});
ln2 = ln2(1)-1;
a2 = 4000;
b2 = 4500;


b_t1 = str2num(char(strrep(B{1,1}(a2:b2,1),'time=',''))); %#ok<*ST2NM>
b_f1 = str2num(char(strrep(B{1,2}(a2:b2,1),'force=','')));
b_po1 = str2num(char(strrep(B{1,3}(a2:b2,1),'position=','')));
b_st1 = str2num(char(strrep(B{1,4}(a2:b2,1),'state=','')));
b_spd1 = -str2num(char(strrep(B{1,5}(a2:b2,1),'speed_cmd=','')));
b_cut = str2num(char(strrep(B{1,6}(a2:b2,1),'current=','')));


b_po2 = 30000 - b_po1;
b_s1 = b_po1*0;


for i=1:(b2-a2)
    b_s1(i) = -100*(b_po1(i+1) - b_po1(i))/(b_t1(i+1) - b_t1(i));
end

b_t2 = b_po1*0;
for i=1:(b2-a2+1)
    b_t2(i) = b_t1(i) - b_t1(1);
end

nonzero = b_po1*0;
n_nonzero = b_po1*0;

for i=10:(b2-a2)
   n_nonzero(i) = sum(sum(nonzero(i-10+1:i)~=0));
end


% plot(b_t2,b_spd1/70);
% hold on;
plot(b_t2,b_f1*20);
hold on;
plot(b_t2,b_po2);
hold on;
% plot(b_t2,b_s1);
% hold on;
% plot(b_t2,b_st1*1000);
% hold on;
plot(b_t2,-b_cut*10);
fclose(fp1);