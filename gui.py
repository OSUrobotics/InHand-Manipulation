#!/usr/bin/env python3

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, QSlider, \
    QLineEdit, QComboBox, QTextEdit, QCheckBox, QRadioButton, QButtonGroup, QDialog
from PyQt5.QtCore import Qt
import json


class GUI(QMainWindow):
    def __init__(self):
        """

        """

        """
        Get all options
        """
        all_options_ss, all_options_act, all_options_rew = self.get_all_options()
        print("LOOK HERE FULL: ss {}, act {}, rew {}".format(all_options_ss, all_options_act, all_options_rew))

        """
        Get default options
        """
        def_options_ss, def_options_act, def_options_rew = self.get_def_options()
        print("LOOK HERE DEF: ss {}, act {}, rew {}".format(def_options_ss, def_options_act, def_options_rew))

        """
        Get current setting
        """
        curr_options_ss, curr_options_act, curr_options_rew = self.get_curr_options()
        print("LOOK HERE CURR: ss {}, act {}, rew {}".format(curr_options_ss, curr_options_act, curr_options_rew))

        """
        Initial Setup
        Create Window
        Set Title
        """
        super(GUI, self).__init__()
        self.setWindowTitle('Set Values')


        """
        Central Widget
        layouts
        Add Widget
        # shape and size
        # sub windows   
        """
        widget = QWidget()
        self.setCentralWidget(widget)

        outside_layout = QVBoxLayout()
        widget.setLayout(outside_layout)

        layout = QHBoxLayout()
        outside_layout.addLayout(layout)

        # Obs space
        obs_layout = QVBoxLayout()
        #obs layout
        obs_heading = QHBoxLayout()
        obs_current = QHBoxLayout()
        obs_default = QHBoxLayout()
        obs_custom = QHBoxLayout()

        obs_layout.addLayout(obs_heading)
        obs_layout.addLayout(obs_current)
        obs_layout.addLayout(obs_default)
        obs_layout.addLayout(obs_custom)
        layout.addLayout(obs_layout)

        # Act space
        act_layout = QVBoxLayout()
        # act layout
        act_heading = QHBoxLayout()
        act_current = QHBoxLayout()
        act_default = QHBoxLayout()
        act_custom = QHBoxLayout()

        act_layout.addLayout(act_heading)
        act_layout.addLayout(act_current)
        act_layout.addLayout(act_default)
        act_layout.addLayout(act_custom)
        layout.addLayout(act_layout)

        # Reward space
        reward_layout = QVBoxLayout()
        # rew layout
        rew_heading = QHBoxLayout()
        rew_current = QHBoxLayout()
        rew_default = QHBoxLayout()
        rew_custom = QHBoxLayout()

        reward_layout.addLayout(rew_heading)
        reward_layout.addLayout(rew_current)
        reward_layout.addLayout(rew_default)
        reward_layout.addLayout(rew_custom)
        layout.addLayout(reward_layout)

        # Radio Button Groups
        ss_group = QButtonGroup(widget)
        act_group = QButtonGroup(widget)
        reward_group = QButtonGroup(widget)

        act_opts_group = QButtonGroup(widget)



        """
        Horizontal separations of all options
        """
        self.ss_label = QLabel('Observation Space:')
        obs_heading.addWidget(self.ss_label)

        self.action_label = QLabel('Action Space:')
        act_heading.addWidget(self.action_label)

        self.reward_label = QLabel('Rewards:')
        rew_heading.addWidget(self.reward_label)

        """
        Observation Space layout
        """
        self.curr_ss_options = QRadioButton('Current State Config:')
        ss_group.addButton(self.curr_ss_options)
        obs_current.addWidget(self.curr_ss_options)

        self.curr_config_ss = QTextEdit("")
        self.show_options(curr_options_ss, self.curr_config_ss)
        self.curr_config_ss.setReadOnly(True)
        obs_current.addWidget(self.curr_config_ss)

        self.default_ss_options = QRadioButton('Default State Config:')
        ss_group.addButton(self.default_ss_options)
        obs_default.addWidget(self.default_ss_options)

        self.def_config_ss = QTextEdit("")
        # print("!!!!!!!!!@@@#######")
        self.show_options(def_options_ss, self.def_config_ss)
        self.def_config_ss.setReadOnly(True)
        obs_default.addWidget(self.def_config_ss)

        self.new_ss_options = QRadioButton('Custom State Config:')
        ss_group.addButton(self.new_ss_options)
        obs_custom.addWidget(self.new_ss_options)

        self.new_ss_text = QTextEdit("")
        self.new_ss_text.setReadOnly(True)
        obs_custom.addWidget(self.new_ss_text)

        """
        Available Options for States, Check Box Style
        """
        self.ss_options = self.create_boxes(all_options_ss, obs_layout, QCheckBox, self.new_ss_text)

        """
        Action Space layout
        """
        self.curr_action_options = QRadioButton('Current Action Config')
        act_group.addButton(self.curr_action_options)
        act_current.addWidget(self.curr_action_options)

        self.curr_config_act = QTextEdit("")
        self.show_options(curr_options_act, self.curr_config_act)
        self.curr_config_act.setReadOnly(True)
        act_current.addWidget(self.curr_config_act)

        self.default_act_options = QRadioButton('Default Action Config:')
        act_group.addButton(self.default_act_options)
        act_default.addWidget(self.default_act_options)

        self.def_config_act = QTextEdit("")
        self.show_options(def_options_act, self.def_config_act)
        self.def_config_act.setReadOnly(True)
        act_default.addWidget(self.def_config_act)

        self.new_act_options = QRadioButton('Custom Action Config:')
        act_group.addButton(self.new_act_options)
        act_custom.addWidget(self.new_act_options)

        self.new_act_text = QTextEdit("")
        act_custom.addWidget(self.new_act_text)

        """
        Available Options for Actions, Radio Button Style
        """
        self.act_options = self.create_boxes(all_options_act, act_layout, QRadioButton, self.new_act_text, act_opts_group)

        """
        Reward Layout
        """
        self.curr_reward_options = QRadioButton('Current Rewards Config')
        reward_group.addButton(self.curr_reward_options)
        rew_current.addWidget(self.curr_reward_options)

        self.curr_config_rew = QTextEdit("")
        self.show_options(curr_options_rew, self.curr_config_rew)
        self.curr_config_rew.setReadOnly(True)
        rew_current.addWidget(self.curr_config_rew)

        self.default_rew_options = QRadioButton('Default Reward Config:')
        reward_group.addButton(self.default_rew_options)
        rew_default.addWidget(self.default_rew_options)

        self.def_config_rew = QTextEdit("")
        self.show_options(def_options_rew, self.def_config_rew)
        self.def_config_rew.setReadOnly(True)
        rew_default.addWidget(self.def_config_rew)

        self.new_rew_options = QRadioButton('Custom Reward Config:')
        reward_group.addButton(self.new_rew_options)
        rew_custom.addWidget(self.new_rew_options)

        self.new_rew_text = QTextEdit("")
        rew_custom.addWidget(self.new_rew_text)

        """
        Available Options for Rewards, Check Box Style
        """
        self.rew_options = self.create_boxes(all_options_rew, reward_layout, QCheckBox, self.new_rew_text)

        """
        Done Button
        """
        self.done_button = QPushButton('OK')
        outside_layout.addWidget(self.done_button)
        self.done_button.clicked.connect(self.update_json_file)

    def get_all_options(self):
        # Read from JSON file
        with open('gym_env_files/envs/all_options.json', 'r') as f:
            all_options = json.load(f)
        all_ss = all_options['full_state_space']
        all_act = all_options['full_action_space']
        all_rew = all_options['full_reward']
        return all_ss, all_act, all_rew

    def get_def_options(self):
        # Read from JSON file
        with open('gym_env_files/envs/rl_spaces_real.json', 'r') as f:
            def_options = json.load(f)
        def_ss = def_options['state_space']
        def_act = def_options['action_space']
        def_rew = def_options['reward']
        return def_ss, def_act, def_rew

    def get_curr_options(self):
        # Read from JSON file
        with open('gym_env_files/envs/rl_spaces.json', 'r') as f:
            curr_options = json.load(f)
        curr_ss = curr_options['state_space']
        curr_act = curr_options['action_space']
        curr_rew = curr_options['reward']
        return curr_ss, curr_act, curr_rew

    def show_options(self, curr_options, TextField):
        for key, value in curr_options.items():
            str_value = str(value)
            TextField.append(key+": "+ str_value)

    def create_boxes(self, options, layout, box_style, new_text_box, group=None):
        options_list = {}
        range_list = []
        i = 0
        for option in options.keys():
            range_layout = QHBoxLayout()
            layout.addLayout(range_layout)
            button = box_style(option)
            options_list.update({button: i})
            if group is not None:
                group.addButton(button)
            range_layout.addWidget(button)
            range_layout.addWidget(QLabel("Min, Max"))
            range_line = QLineEdit("")
            range_list.append(range_line)
            range_layout.addWidget(range_line)
            range_line.setReadOnly(True)
            button.toggled.connect(lambda: self.add_option(new_text_box, layout, options_list, range_list))
            range_line.editingFinished.connect(lambda: self.add_range(options_list, range_list, new_text_box))
            i += 1
        print("RANGE_LIST: {}, i: {}".format(range_list, i))
        return options_list

    def add_option(self, new_text, layout, options_dict, min_max_text_list):
        option = self.sender()
        min_max_text_index = options_dict[option]
        if option.isChecked():
            min_max_text_list[min_max_text_index].setReadOnly(False)
            # Check if it exists in textbox
                # if it does, Don't add box
            # else do the following:
            # range_layout = QHBoxLayout()
            # layout.addLayout(range_layout)
            # range_layout.addWidget(QLabel("Min, Max"))
            # range_line = QLineEdit("")
            # range_layout.addWidget(range_line)
            # range_line.editingFinished.connect(lambda: self.add_range(option, new_text))

        else:
            # Check if it exists in textbox
            print("OPTION BOX TEXT: {}".format(option.text()))
            print("TEXT BOX TEXT: {}".format(new_text.toPlainText()))
            if option.text() in new_text.toPlainText():
                # find text
                find_in_text = new_text.toPlainText()
                found_start = find_in_text.find(option.text())
                found_end = find_in_text[found_start:].find("]")
                text_to_remove = find_in_text[found_start: found_start + found_end + 1]
                print("FOUND!: {}".format(text_to_remove))
                # Remove from textfield
                try:
                    front_part = find_in_text[:found_start]
                    back_part = find_in_text[found_start + found_end + 2:]
                    replace_above = front_part + back_part
                except IndexError:
                    replace_above = ""
                    front_part = find_in_text[:found_start]
                    back_part = find_in_text[found_start + found_end:]
                print("REPLACE: {} Front: {} num: {} Back: {} num: {}".format(replace_above, front_part, found_start,
                                                                              back_part, found_end))
                new_text.setText(replace_above)
            min_max_text_list[min_max_text_index].setText("")
            min_max_text_list[min_max_text_index].setReadOnly(True)
            pass

    def add_range(self, option_dict, min_max_list, text_box):
        range_text_box = self.sender()
        get_index = min_max_list.index(range_text_box)
        print("KEY: {}, INDEX: {}, DICT {}".format(range_text_box, get_index, option_dict))
        for key, value in option_dict.items():
            if value == get_index:
                option_box = key
        range_text = range_text_box.text()
        name_text = option_box.text()
        if name_text not in text_box.toPlainText():
            final_text = name_text+": ["+range_text+"]"

            if option_box.isChecked():
                text_box.append(final_text)
            range_text_box.setReadOnly(True)


    def update_json_file(self):
        json_dict = {}
        if self.curr_ss_options.isChecked():
            text_to_add_ss = self.curr_config_ss.toPlainText().split('\n')
        elif self.default_ss_options.isChecked():
            text_to_add_ss = self.def_config_ss.toPlainText().split('\n')
        elif self.new_ss_options.isChecked():
            text_to_add_ss = self.new_ss_text.toPlainText().split('\n')
        else:
            text_to_add_ss = "Please select an option"

        new_text_to_add_ss = self.convert_to_dict(text_to_add_ss)

        if self.curr_action_options.isChecked():
            text_to_add_act = self.curr_config_act.toPlainText().split('\n')
        elif self.default_act_options.isChecked():
            text_to_add_act = self.def_config_act.toPlainText().split('\n')
        elif self.new_act_options.isChecked():
            text_to_add_act = self.new_act_text.toPlainText().split('\n')
        else:
            text_to_add_act = "Please select an option"

        new_text_to_add_act = self.convert_to_dict(text_to_add_act)

        if self.curr_reward_options.isChecked():
            text_to_add_rew = self.curr_config_rew.toPlainText().split('\n')
        elif self.default_rew_options.isChecked():
            text_to_add_rew = self.def_config_rew.toPlainText().split('\n')
        elif self.new_rew_options.isChecked():
            text_to_add_rew = self.new_rew_text.toPlainText().split('\n')
        else:
            text_to_add_rew = "Please select an option"

        new_text_to_add_rew = self.convert_to_dict(text_to_add_rew)

        json_dict.update({"state_space": new_text_to_add_ss})
        json_dict.update({"action_space": new_text_to_add_act})
        json_dict.update({"reward": new_text_to_add_rew})
        # Write from JSON file
        with open('gym_env_files/envs/rl_spaces.json', 'w') as f:
            json.dump(json_dict, f)
        self.close()

    def convert_to_dict(self, text_to_add):
        new_dict = {}
        print("FULL TEXT:", text_to_add)
        for text in text_to_add:
            new_text = text.split(":")
            print("NEW TEXT:", new_text)
            new_text[1] = new_text[1].replace('[','')
            new_text[1] = new_text[1].replace(']', '')
            float_vals = new_text[1].split(",")
            print("FLOAT VALS:", float_vals)
            for vals in range(0, len(float_vals)):
                float_vals[vals] = float(float_vals[vals])
            new_dict.update({new_text[0]: float_vals})
        # print("Text to add: {} \nKey value split: {} \nfloats: {}\n".format(text_to_add, new_text, float_vals))
        return new_dict

if __name__ == '__main__':
    app = QApplication([])

    interface = GUI()

    interface.show()

    app.exec_()