#pragma once

#include "main.h"

#include <cstring>

namespace cmd
{
    namespace modbus
    {
        enum param_type
        {
            input,
            holding
        };

        template<typename T> constexpr size_t get_modbus_size()
        {
            return sizeof(T) / sizeof(uint16_t) + (sizeof(T) % sizeof(uint16_t) != 0 ? 1u : 0u);
        }
        
        template<typename T> class param
        {
        public:
            param(T* p, const char* n, param_type t);
            ~param();

            constexpr size_t get_size(); //In modbus registers, i.e. 16-bit uint

            const char* get_name();
            T get();
            void set_reg(size_t i, uint16_t v);
            const uint16_t *get_reg(size_t i);
            void set(T v);
        protected:
            bool internal_storage = false;
            T* ptr;
            const char* name;
            param_type type;
            uint16_t regs[];
        };

        template<typename T> class param_array
        {
        private:
            size_t capacity;
            size_t index = 0;

            param<T>* arr;
        public:
            param_array(size_t c);
            ~param_array();

            const param<T>* get(size_t i);
            const param<T>* add(T* p, const char* n, param_type t);
            size_t length();
            uint16_t get_reg(size_t i);
            void set_reg(size_t i, uint16_t v);
        };

        class db
        {
        private: //Perceived layout
            param_array<uint16_t>* holding_uint;
            param_array<float>* holding_float;

            param_array<uint16_t>* input_uint;
            param_array<float>* input_float;
        public:
            db(size_t c_hold_uint, size_t c_hold_float, size_t c_in_uint, size_t c_in_float);
            ~db();

            template<typename T> const param<T>* add_internal(param_type t, const char* name);
            template<typename T> const param<T>* add_external_input(const T* p, const char* name);
            template<typename T> const param<T>* add_external_holding(T* p, const char* name);

            uint16_t read_input_reg(size_t i);
            uint16_t read_holding_reg(size_t i);
            void write_holding_reg(size_t i, uint16_t v);
        };
        
        db::db(size_t c_hold_uint, size_t c_hold_float, size_t c_in_uint, size_t c_in_float)
        {
            holding_uint = new param_array<uint16_t>(c_hold_uint);
            holding_float = new param_array<float>(c_hold_float);
            input_uint = new param_array<uint16_t>(c_in_uint);
            input_float = new param_array<float>(c_in_float);
        }
        db::~db()
        {
            delete holding_uint;
            delete holding_float;
            delete input_uint;
            delete input_float;
        }

        template <typename T>
        param<T>::param(T *p, const char *n, param_type t) : name(n), type(t)
        {
            if (p == NULL)
            {
                p = new T();
                internal_storage = true;
            }
            ptr = p;
        }
        template <typename T>
        param<T>::~param()
        {
            if (internal_storage)
            {
                delete ptr;
            }
        }
        template <typename T>
        constexpr size_t param<T>::get_size()
        {
            return get_modbus_size<T>();
        }
        template <typename T>
        T param<T>::get() { return *ptr; }
        template <typename T>
        const char *param<T>::get_name() { return name; }
        template <typename T>
        const uint16_t *param<T>::get_reg(size_t i)
        {
            memcpy(regs, ptr, sizeof(T));
            const size_t pad = (get_size() * sizeof(uint16_t)) % sizeof(T);
            memset(regs + sizeof(T), 0, pad);
            return &(regs[i]);
        }
        template <typename T>
        void param<T>::set_reg(size_t i, uint16_t v)
        {
            assert_param(type == param_type::holding);

            regs[i] = v;
            if ((i + 1) == get_size())
            {
                T b;
                memcpy(&b, regs, sizeof(b));
                *ptr = b;
            }
        }
        template <typename T>
        void param<T>::set(T v)
        {
            *ptr = v;
        }

        template <typename T>
        param_array<T>::param_array(size_t c) : capacity(c) {}
        template <typename T>
        param_array<T>::~param_array()
        {
            for (size_t i = 0; i < index; i++)
            {
                delete (arr + i);
            }
        }
        template <typename T>
        const param<T> *param_array<T>::get(size_t i)
        {
            return arr + i;
        }
        template <typename T>
        const param<T> *param_array<T>::add(T *p, const char *n, param_type t)
        {
            assert_param(capacity > index);

            auto b = arr + index++;
            b = new param<T>(p, n, t);

            return b;
        }
        template <typename T>
        size_t param_array<T>::length()
        {
            return index;
        }
        template <typename T>
        uint16_t param_array<T>::get_reg(size_t i)
        {
            param<T> instance = arr[i / get_modbus_size<T>()];
            return *instance.get_reg(i % get_modbus_size<T>());
        }
        template <typename T>
        void param_array<T>::set_reg(size_t i, uint16_t v)
        {
            param<T> instance = arr[i / get_modbus_size<T>()];
            instance.set_reg(i % get_modbus_size<T>(), v);
        }

        template<typename T> const param<T>* db::add_internal(param_type t, const char* name) = delete;
        template <> const param<uint16_t>* db::add_internal<uint16_t>(param_type t, const char* name)
        {
            switch (t)
            {
            case param_type::holding:
                return holding_uint->add(NULL, name, t);
            case param_type::input:
                return input_uint->add(NULL, name, t);
            default:
                assert_param(0);
            }
            return NULL;
        }
        template <> const param<float>* db::add_internal<float>(param_type t, const char* name)
        {
            switch (t)
            {
            case param_type::holding:
                return holding_float->add(NULL, name, t);
            case param_type::input:
                return input_float->add(NULL, name, t);
            default:
                assert_param(0);
            }
            return NULL;
        }
        template<typename T> const param<T>* db::add_external_holding(T* p, const char* name) = delete;
        template<typename T> const param<T>* db::add_external_input(const T* p, const char* name) = delete;
        template <> const param<uint16_t>* db::add_external_holding<uint16_t>(uint16_t* p, const char* name)
        {
            return holding_uint->add(p, name, param_type::holding);
        }
        template <> const param<float>* db::add_external_holding<float>(float* p, const char* name)
        {
            return holding_float->add(p, name, param_type::holding);
        }
        template <> const param<uint16_t>* db::add_external_input<uint16_t>(const uint16_t* p, const char* name)
        {
            return input_uint->add(const_cast<uint16_t*>(p), name, param_type::input);
        }
        template <> const param<float>* db::add_external_input<float>(const float* p, const char* name)
        {
            return input_float->add(const_cast<float*>(p), name, param_type::input);
        }
        uint16_t db::read_input_reg(size_t i)
        {
            assert_param(i < (input_uint->length() + input_float->length()));

            if (i < input_uint->length())
            {
                return input_uint->get_reg(i);
            }
            else
            {
                i -= input_uint->length();
                return input_float->get_reg(i);
            }
        }
        uint16_t db::read_holding_reg(size_t i)
        {
            assert_param(i < (holding_uint->length() + holding_float->length()));

            if (i < holding_uint->length())
            {
                return holding_uint->get_reg(i);
            }
            else
            {
                i -= holding_uint->length();
                return holding_float->get_reg(i);
            }
        }
        void db::write_holding_reg(size_t i, uint16_t v)
        {
            assert_param(i < (holding_uint->length() + holding_float->length()));

            if (i < holding_uint->length())
            {
                holding_uint->set_reg(i, v);
            }
            else
            {
                i -= holding_uint->length();
                holding_float->set_reg(i, v);
            }
        }
    }
}