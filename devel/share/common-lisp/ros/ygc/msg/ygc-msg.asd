
(cl:in-package :asdf)

(defsystem "ygc-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GroupBearing" :depends-on ("_package_GroupBearing"))
    (:file "_package_GroupBearing" :depends-on ("_package"))
    (:file "Bearing2D" :depends-on ("_package_Bearing2D"))
    (:file "_package_Bearing2D" :depends-on ("_package"))
  ))