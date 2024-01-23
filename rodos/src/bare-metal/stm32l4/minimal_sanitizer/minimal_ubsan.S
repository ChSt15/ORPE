.syntax unified
    .cpu cortex-m4
    .fpu softvfp
    .thumb

.section .text.__ubsan_handle_type_mismatch
.weak __ubsan_handle_type_mismatch
.weak __ubsan_handle_type_mismatch_v1
.type __ubsan_handle_type_mismatch, %function
.type __ubsan_handle_type_mismatch_v1, %function
__ubsan_handle_type_mismatch:
__ubsan_handle_type_mismatch_v1:
    bkpt
.size __ubsan_handle_type_mismatch, .-__ubsan_handle_type_mismatch



.section .text.__ubsan_handle_alignment_assumption
.weak __ubsan_handle_alignment_assumption
.weak __ubsan_handle_alignment_assumption_v1
.type __ubsan_handle_alignment_assumption, %function
.type __ubsan_handle_alignment_assumption_v1, %function
__ubsan_handle_alignment_assumption:
__ubsan_handle_alignment_assumption_v1:
    bkpt
.size __ubsan_handle_alignment_assumption, .-__ubsan_handle_alignment_assumption


.section .text.__ubsan_handle_add_overflow
.weak __ubsan_handle_add_overflow
.weak __ubsan_handle_add_overflow_v1
.type __ubsan_handle_add_overflow, %function
.type __ubsan_handle_add_overflow_v1, %function
__ubsan_handle_add_overflow:
__ubsan_handle_add_overflow_v1:
    bkpt
.size __ubsan_handle_add_overflow, .-__ubsan_handle_add_overflow



.section .text.__ubsan_handle_sub_overflow
.weak __ubsan_handle_sub_overflow
.weak __ubsan_handle_sub_overflow_v1
.type __ubsan_handle_sub_overflow, %function
.type __ubsan_handle_sub_overflow_v1, %function
__ubsan_handle_sub_overflow:
__ubsan_handle_sub_overflow_v1:
    bkpt
.size __ubsan_handle_sub_overflow, .-__ubsan_handle_sub_overflow



.section .text.__ubsan_handle_mul_overflow
.weak __ubsan_handle_mul_overflow
.weak __ubsan_handle_mul_overflow_v1
.type __ubsan_handle_mul_overflow, %function
.type __ubsan_handle_mul_overflow_v1, %function
__ubsan_handle_mul_overflow:
__ubsan_handle_mul_overflow_v1:
    bkpt
.size __ubsan_handle_mul_overflow, .-__ubsan_handle_mul_overflow



.section .text.__ubsan_handle_negate_overflow
.weak __ubsan_handle_negate_overflow
.weak __ubsan_handle_negate_overflow_v1
.type __ubsan_handle_negate_overflow, %function
.type __ubsan_handle_negate_overflow_v1, %function
__ubsan_handle_negate_overflow:
__ubsan_handle_negate_overflow_v1:
    bkpt
.size __ubsan_handle_negate_overflow, .-__ubsan_handle_negate_overflow



.section .text.__ubsan_handle_divrem_overflow
.weak __ubsan_handle_divrem_overflow
.weak __ubsan_handle_divrem_overflow_v1
.type __ubsan_handle_divrem_overflow, %function
.type __ubsan_handle_divrem_overflow_v1, %function
__ubsan_handle_divrem_overflow:
__ubsan_handle_divrem_overflow_v1:
    bkpt
.size __ubsan_handle_divrem_overflow, .-__ubsan_handle_divrem_overflow



.section .text.__ubsan_handle_shift_out_of_bounds
.weak __ubsan_handle_shift_out_of_bounds
.weak __ubsan_handle_shift_out_of_bounds_v1
.type __ubsan_handle_shift_out_of_bounds, %function
.type __ubsan_handle_shift_out_of_bounds_v1, %function
__ubsan_handle_shift_out_of_bounds:
__ubsan_handle_shift_out_of_bounds_v1:
    bkpt
.size __ubsan_handle_shift_out_of_bounds, .-__ubsan_handle_shift_out_of_bounds



.section .text.__ubsan_handle_out_of_bounds
.weak __ubsan_handle_out_of_bounds
.weak __ubsan_handle_out_of_bounds_v1
.type __ubsan_handle_out_of_bounds, %function
.type __ubsan_handle_out_of_bounds_v1, %function
__ubsan_handle_out_of_bounds:
__ubsan_handle_out_of_bounds_v1:
    bkpt
.size __ubsan_handle_out_of_bounds, .-__ubsan_handle_out_of_bounds



.section .text.__ubsan_handle_builtin_unreachable
.weak __ubsan_handle_builtin_unreachable
.weak __ubsan_handle_builtin_unreachable_v1
.type __ubsan_handle_builtin_unreachable, %function
.type __ubsan_handle_builtin_unreachable_v1, %function
__ubsan_handle_builtin_unreachable:
__ubsan_handle_builtin_unreachable_v1:
    bkpt
.size __ubsan_handle_builtin_unreachable, .-__ubsan_handle_builtin_unreachable



.section .text.__ubsan_handle_missing_return
.weak __ubsan_handle_missing_return
.weak __ubsan_handle_missing_return_v1
.type __ubsan_handle_missing_return, %function
.type __ubsan_handle_missing_return_v1, %function
__ubsan_handle_missing_return:
__ubsan_handle_missing_return_v1:
    bkpt
.size __ubsan_handle_missing_return, .-__ubsan_handle_missing_return



.section .text.__ubsan_handle_vla_bound_not_positive
.weak __ubsan_handle_vla_bound_not_positive
.weak __ubsan_handle_vla_bound_not_positive_v1
.type __ubsan_handle_vla_bound_not_positive, %function
.type __ubsan_handle_vla_bound_not_positive_v1, %function
__ubsan_handle_vla_bound_not_positive:
__ubsan_handle_vla_bound_not_positive_v1:
    bkpt
.size __ubsan_handle_vla_bound_not_positive, .-__ubsan_handle_vla_bound_not_positive



.section .text.__ubsan_handle_float_cast_overflow
.weak __ubsan_handle_float_cast_overflow
.weak __ubsan_handle_float_cast_overflow_v1
.type __ubsan_handle_float_cast_overflow, %function
.type __ubsan_handle_float_cast_overflow_v1, %function
__ubsan_handle_float_cast_overflow:
__ubsan_handle_float_cast_overflow_v1:
    bkpt
.size __ubsan_handle_float_cast_overflow, .-__ubsan_handle_float_cast_overflow



.section .text.__ubsan_handle_load_invalid_value
.weak __ubsan_handle_load_invalid_value
.weak __ubsan_handle_load_invalid_value_v1
.type __ubsan_handle_load_invalid_value, %function
.type __ubsan_handle_load_invalid_value_v1, %function
__ubsan_handle_load_invalid_value:
__ubsan_handle_load_invalid_value_v1:
    bkpt
.size __ubsan_handle_load_invalid_value, .-__ubsan_handle_load_invalid_value



.section .text.__ubsan_handle_invalid_builtin
.weak __ubsan_handle_invalid_builtin
.weak __ubsan_handle_invalid_builtin_v1
.type __ubsan_handle_invalid_builtin, %function
.type __ubsan_handle_invalid_builtin_v1, %function
__ubsan_handle_invalid_builtin:
__ubsan_handle_invalid_builtin_v1:
    bkpt
.size __ubsan_handle_invalid_builtin, .-__ubsan_handle_invalid_builtin



.section .text.__ubsan_handle_function_type_mismatch
.weak __ubsan_handle_function_type_mismatch
.weak __ubsan_handle_function_type_mismatch_v1
.type __ubsan_handle_function_type_mismatch, %function
.type __ubsan_handle_function_type_mismatch_v1, %function
__ubsan_handle_function_type_mismatch:
__ubsan_handle_function_type_mismatch_v1:
    bkpt
.size __ubsan_handle_function_type_mismatch, .-__ubsan_handle_function_type_mismatch



.section .text.__ubsan_handle_implicit_conversion
.weak __ubsan_handle_implicit_conversion
.weak __ubsan_handle_implicit_conversion_v1
.type __ubsan_handle_implicit_conversion, %function
.type __ubsan_handle_implicit_conversion_v1, %function
__ubsan_handle_implicit_conversion:
__ubsan_handle_implicit_conversion_v1:
    bkpt
.size __ubsan_handle_implicit_conversion, .-__ubsan_handle_implicit_conversion



.section .text.__ubsan_handle_nonnull_arg
.weak __ubsan_handle_nonnull_arg
.weak __ubsan_handle_nonnull_arg_v1
.type __ubsan_handle_nonnull_arg, %function
.type __ubsan_handle_nonnull_arg_v1, %function
__ubsan_handle_nonnull_arg:
__ubsan_handle_nonnull_arg_v1:
    bkpt
.size __ubsan_handle_nonnull_arg, .-__ubsan_handle_nonnull_arg



.section .text.__ubsan_handle_nonnull_return
.weak __ubsan_handle_nonnull_return
.weak __ubsan_handle_nonnull_return_v1
.type __ubsan_handle_nonnull_return, %function
.type __ubsan_handle_nonnull_return_v1, %function
__ubsan_handle_nonnull_return:
__ubsan_handle_nonnull_return_v1:
    bkpt
.size __ubsan_handle_nonnull_return, .-__ubsan_handle_nonnull_return



.section .text.__ubsan_handle_nullability_arg
.weak __ubsan_handle_nullability_arg
.weak __ubsan_handle_nullability_arg_v1
.type __ubsan_handle_nullability_arg, %function
.type __ubsan_handle_nullability_arg_v1, %function
__ubsan_handle_nullability_arg:
__ubsan_handle_nullability_arg_v1:
    bkpt
.size __ubsan_handle_nullability_arg, .-__ubsan_handle_nullability_arg



.section .text.__ubsan_handle_nullability_return
.weak __ubsan_handle_nullability_return
.weak __ubsan_handle_nullability_return_v1
.type __ubsan_handle_nullability_return, %function
.type __ubsan_handle_nullability_return_v1, %function
__ubsan_handle_nullability_return:
__ubsan_handle_nullability_return_v1:
    bkpt
.size __ubsan_handle_nullability_return, .-__ubsan_handle_nullability_return



.section .text.__ubsan_handle_pointer_overflow
.weak __ubsan_handle_pointer_overflow
.weak __ubsan_handle_pointer_overflow_v1
.type __ubsan_handle_pointer_overflow, %function
.type __ubsan_handle_pointer_overflow_v1, %function
__ubsan_handle_pointer_overflow:
__ubsan_handle_pointer_overflow_v1:
    bkpt
.size __ubsan_handle_pointer_overflow, .-__ubsan_handle_pointer_overflow



.section .text.__ubsan_handle_cfi_check_fail
.weak __ubsan_handle_cfi_check_fail
.weak __ubsan_handle_cfi_check_fail_v1
.type __ubsan_handle_cfi_check_fail, %function
.type __ubsan_handle_cfi_check_fail_v1, %function
__ubsan_handle_cfi_check_fail:
__ubsan_handle_cfi_check_fail_v1:
    bkpt
.size __ubsan_handle_cfi_check_fail, .-__ubsan_handle_cfi_check_fail
