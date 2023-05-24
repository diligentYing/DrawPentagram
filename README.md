# DrawPentagram
matlab+Adams，实现在matlab和Adams里机器人末端画出五角星。
# requirement
matlab中添加机器人工具箱
# Run
先运行plotFiveStar.m得到五角星五个顶点坐标，再运行KUKA_R700.m在matlab里绘制五角星并导出关节角数据为 *.txt文件，*.txt文件导入到Adams里，实现Adams里绘制五角星（具体实现看.mp4文件）。
