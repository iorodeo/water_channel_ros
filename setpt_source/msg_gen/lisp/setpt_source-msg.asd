
(cl:in-package :asdf)

(defsystem "setpt_source-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "SetptMsg" :depends-on ("_package_SetptMsg"))
    (:file "_package_SetptMsg" :depends-on ("_package"))
  ))