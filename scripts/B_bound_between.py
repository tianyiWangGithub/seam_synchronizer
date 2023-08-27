import matplotlib.pyplot as plt
import numpy as np

import matplotlib.pyplot as plt


# 读取数据
data = np.loadtxt('/home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/B/success_rate.txt')
data_ct = np.loadtxt("/home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/B/data_ct_B.txt")

# 提取数据中的两列
data_1 = data[:, 1]
data_2 = data[:, 0]
data1 = [i*100 for i in data_1]
data2 = [i*100 for i in data_2]
data1_ct = data_ct[:, 0]
data2_ct = data_ct[:, 1]

# 创建绘图对象和子图对象
fig, ax = plt.subplots(figsize=(10,6))

# 定义每个柱子的横坐标位置和宽度
xticks = np.array([1,2,3, 4, 5, 6, 7, 8, 9,10])
width = 0.4

# 绘制柱形图
ax.bar(xticks - width/2, data1, width, color='lightcoral', label='SEAM')
ax.bar(xticks + width/2, data2, width, color='lightskyblue', label='ApproximateTime')

# 设置横坐标和纵坐标的范围和标签
ax.set_xlim(0, 11)
ax.set_ylim(0, 150)
ax.set_xticks(xticks)
ax.set_xticklabels(['100','105','110', '115', '120','125','130','135','140','145'],size='30')
ax.set_xlabel('B(ms)',size='30')
ax.set_ylabel('Succ Rate(%)',size='30')
#ax.set_yticklabels(ax.get_yticks(), size='30')
ax.set_yticklabels(['0','20', '40', '60', '80', '100'],size='30')
ax.set_yticks([0, 20, 40, 60, 80,100])

# 添加坐标轴边框
ax.spines['top'].set_visible(True)
ax.spines['right'].set_visible(True)
ax.spines['bottom'].set_visible(True)
ax.spines['left'].set_visible(True)

# 绘制折线图
ax2 = ax.twinx()  # 创建一个共享x轴的新y轴

ax2.plot(xticks, data1_ct, marker='s', color='red', linestyle='-', label='SEAM',linewidth=5,markersize=15)
ax2.plot(xticks, data2_ct, marker='x', color='blue', linestyle='-', label='ApproximateTime',linewidth=5,markersize=20)

ax2.set_ylim(0, 180)
ax2.set_ylabel('Comp Time(μs)',size='30')  # 设置第二个y轴的标签
#ax2.set_yticklabels(ax2.get_yticks(), size='30')
ax2.set_yticklabels(['0','20', '40', '60', '80', '100','120'],size='30')

ax2.set_yticks([0, 20, 40, 60, 80,100,120])


# 添加第一个图例
legend1 = ax.legend(prop={'size': 30}, bbox_to_anchor=(0.2, 1), loc='upper center', frameon=True, borderaxespad=0.5)
legend1.set_title('Succ Rate')
legend1.get_title().set_fontsize(30)

# 添加第二个图例
legend2 = ax2.legend(prop={'size': 30}, bbox_to_anchor=(0.8, 1), loc='upper center', frameon=True, borderaxespad=0.5)
legend2.set_title('Comp Time')
legend2.get_title().set_fontsize(30)

title1 = legend1.get_title()
title1.set_bbox({'facecolor': '#FFFACD', 'edgecolor': 'red', 'pad': 8,'linewidth': 3})
title2 = legend2.get_title()
title2.set_bbox({'facecolor': '#FFFACD', 'edgecolor': 'red', 'pad': 8,'linewidth': 3})

legend_frame1 = legend1.get_frame()
legend_frame1.set_facecolor('#FFFACD')
legend_frame2 = legend2.get_frame()
legend_frame2.set_facecolor('#FFFACD')


fig.subplots_adjust(left=0.15, right=0.85)

# 显示图形
plt.savefig('seam_apr_B', dpi=400)
plt.show()