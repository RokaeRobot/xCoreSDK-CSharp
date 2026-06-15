using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace xCoreSDK_CSharp
{
    public partial class DemoForm : Form
    {
        private MainForm mainForm;

        public DemoForm()
        {
            InitializeComponent();
        }

        public bool[] opened = { false, false };
        private void tabControl_Demo_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (!opened[tabControl_Demo.SelectedIndex])
            {
                string? formName = ((TabControl)sender).SelectedTab.Tag as string;
                InitializeForm(formName, sender);
            }
        }

        private void DemoForm_Load(object sender, EventArgs e)
        {
            mainForm = new MainForm();
            mainForm.FormBorderStyle = FormBorderStyle.None;
            mainForm.TopLevel = false;
            mainForm.Parent = tabControl_Demo.SelectedTab;
            mainForm.ControlBox = false;
            mainForm.Dock = DockStyle.Fill;
            mainForm.Show();
            opened[tabControl_Demo.SelectedIndex] = true;
        }

        private void InitializeForm(string formName, object sender)
        {
            Form form = (Form)Assembly.GetExecutingAssembly().CreateInstance(formName);
            form.FormBorderStyle = FormBorderStyle.None;
            form.TopLevel = false;
            form.Parent = ((TabControl)sender).SelectedTab;
            form.ControlBox = false;
            form.Dock = DockStyle.Fill;
            form.Show();
            opened[tabControl_Demo.SelectedIndex] = true;

            ((FormInterface)form).setRobotInstance(mainForm.RobotDemo);
        }
    }
}
