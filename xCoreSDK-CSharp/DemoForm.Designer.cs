namespace xCoreSDK_CSharp
{
    partial class DemoForm
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            tabPage_Application = new TabPage();
            tabPage_Main = new TabPage();
            tabControl_Demo = new TabControl();
            tabControl_Demo.SuspendLayout();
            SuspendLayout();
            // 
            // tabPage_Application
            // 
            tabPage_Application.Location = new Point(4, 28);
            tabPage_Application.Margin = new Padding(2);
            tabPage_Application.Name = "tabPage_Application";
            tabPage_Application.Padding = new Padding(2);
            tabPage_Application.Size = new Size(1041, 540);
            tabPage_Application.TabIndex = 1;
            tabPage_Application.Tag = "xCoreSDK_CSharp.ApplicationForm";
            tabPage_Application.Text = "更多示例";
            tabPage_Application.UseVisualStyleBackColor = true;
            // 
            // tabPage_Main
            // 
            tabPage_Main.Location = new Point(4, 28);
            tabPage_Main.Margin = new Padding(2);
            tabPage_Main.Name = "tabPage_Main";
            tabPage_Main.Padding = new Padding(2);
            tabPage_Main.Size = new Size(1041, 540);
            tabPage_Main.TabIndex = 0;
            tabPage_Main.Tag = "xCoreSDK_CSharp.MainForm";
            tabPage_Main.Text = "主界面";
            tabPage_Main.UseVisualStyleBackColor = true;
            // 
            // tabControl_Demo
            // 
            tabControl_Demo.Controls.Add(tabPage_Main);
            tabControl_Demo.Controls.Add(tabPage_Application);
            tabControl_Demo.Dock = DockStyle.Fill;
            tabControl_Demo.Font = new Font("Microsoft YaHei UI", 10F, FontStyle.Regular, GraphicsUnit.Point);
            tabControl_Demo.Location = new Point(0, 0);
            tabControl_Demo.Margin = new Padding(2);
            tabControl_Demo.Name = "tabControl_Demo";
            tabControl_Demo.SelectedIndex = 0;
            tabControl_Demo.Size = new Size(1049, 572);
            tabControl_Demo.TabIndex = 0;
            tabControl_Demo.SelectedIndexChanged += tabControl_Demo_SelectedIndexChanged;
            // 
            // DemoForm
            // 
            AutoScaleDimensions = new SizeF(10F, 21F);
            AutoScaleMode = AutoScaleMode.Font;
            ClientSize = new Size(1049, 572);
            Controls.Add(tabControl_Demo);
            Margin = new Padding(2);
            Name = "DemoForm";
            Text = "xCoreSDK Demo";
            Load += DemoForm_Load;
            tabControl_Demo.ResumeLayout(false);
            ResumeLayout(false);
        }

        #endregion

        private TabPage tabPage_Application;
        private TabPage tabPage_Main;
        private TabControl tabControl_Demo;
    }
}