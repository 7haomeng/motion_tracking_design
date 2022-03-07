
(cl:in-package :asdf)

(defsystem "LKOpticalFlow_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Points" :depends-on ("_package_Points"))
    (:file "_package_Points" :depends-on ("_package"))
  ))