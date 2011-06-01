
(cl:in-package :asdf)

(defsystem "setpt_source-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "SetptActionGoal" :depends-on ("_package_SetptActionGoal"))
    (:file "_package_SetptActionGoal" :depends-on ("_package"))
    (:file "SetptActionResult" :depends-on ("_package_SetptActionResult"))
    (:file "_package_SetptActionResult" :depends-on ("_package"))
    (:file "SetptMsg" :depends-on ("_package_SetptMsg"))
    (:file "_package_SetptMsg" :depends-on ("_package"))
    (:file "SetptActionFeedback" :depends-on ("_package_SetptActionFeedback"))
    (:file "_package_SetptActionFeedback" :depends-on ("_package"))
    (:file "SetptGoal" :depends-on ("_package_SetptGoal"))
    (:file "_package_SetptGoal" :depends-on ("_package"))
    (:file "SetptAction" :depends-on ("_package_SetptAction"))
    (:file "_package_SetptAction" :depends-on ("_package"))
    (:file "SetptFeedback" :depends-on ("_package_SetptFeedback"))
    (:file "_package_SetptFeedback" :depends-on ("_package"))
    (:file "SetptResult" :depends-on ("_package_SetptResult"))
    (:file "_package_SetptResult" :depends-on ("_package"))
  ))