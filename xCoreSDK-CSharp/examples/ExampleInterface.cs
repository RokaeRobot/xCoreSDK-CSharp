using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using rokae.clr;

namespace xCoreSDK_CSharp.examples
{
    public interface ExampleInterface
    {
        public BaseRobot RobotInst { get; }
        public Cobot CobotInst {  get; }
        public void Demo_JogRobot(JogSpace space, double rate, double step, bool diretion, int index, ref ErrorCode ec)
        {
            ErrorCode _e;
            RobotInst.startJog(space, rate, step, (uint)index, diretion, out _e);
            ec = _e;
        }

        public string Demo_JointMove();
        public string Demo_StartRecordingPos();
        public string Demo_StopRecordingPos();
    }
}
