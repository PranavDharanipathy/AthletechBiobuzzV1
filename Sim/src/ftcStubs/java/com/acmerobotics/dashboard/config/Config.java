package com.acmerobotics.dashboard.config;

import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;

//stand-in for FTC Dashboard's @Config
@Retention(RetentionPolicy.RUNTIME)
public @interface Config {

    String value() default "";
}
