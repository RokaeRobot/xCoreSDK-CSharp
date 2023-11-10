using rokae.clr;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Net.NetworkInformation;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace xCoreSDK_CSharp
{
    /// <summary>
    /// 坐标系标定(N点标定)接口示例
    /// </summary>
    /// <remarks>
    /// 各坐标系类型支持的标定方法及注意事项：
    ///   1) 工具坐标系: 三点/四点/六点标定法
    ///   2) 工件坐标系: 三点标定。标定结果不会相对用户坐标系做变换。
    ///   3) 基坐标系: 六点标定。标定前请确保动力学约束和前馈已关闭。
    ///   若标定成功(无错误码)，控制器会自动保存标定结果，重启控制器后生效。
    /// </remarks>
    class CalibrateFrame
    {
       public CalibrateFrame(xMateRobot robot, FrameType type, int point_num, bool is_held)
        {
            Robot = robot;
            FrameType = type;
            IsHeld = is_held;
            PointList = new List<double[]>(point_num);
            BaseAux = new double[3];
        }

        /// <summary>
        /// 标定方法参考: Jog或拖动机械臂到某个位置，读取当前轴角度
        /// </summary>
        /// <param name="point_index"></param>
        public void SetPoint(int point_index)
        {
            if (point_index >= PointList.Capacity)
            {
                Console.WriteLine("over range");
                return;
            }
            ErrorCode ec;
            // please guarantee robot is valid before calling
            PointList.Add(Robot.jointPos(out ec));
        }

        /// <summary>
        /// 标定结果
        /// </summary>
        /// <param name="ec">当错误码没有被置位时，标定结果有效</param>
        /// <returns>坐标系和误差</returns>
        public FrameCalibrationResult confirm(out ErrorCode ec)
        {
            return Robot.calibrateFrame(FrameType, PointList, IsHeld, BaseAux, out ec);
        }


        private xMateRobot Robot;
        
        /// <summary>
        /// 坐标系类型，支持工具(FrameType::tool), 工件(FrameType::wobj), 基坐标系(FrameType::base)
        /// </summary>
        private FrameType FrameType;
        
        /// <summary>
        /// 轴角度列表，列表长度为N。例如，使用三点法标定工具坐标系，应传入3组轴角度。轴角度的单位是弧度。
        /// </summary>
        private List<double[]> PointList;
        
        /// <summary>
        /// 基坐标系标定时用到的辅助点, 单位[米]
        /// </summary>
        public double[] BaseAux;
        
        /// <summary>
        /// true - 机器人手持 | false - 外部。仅影响工具/工件的标定
        /// </summary>
        private bool IsHeld;
    }
}
