from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.chassis_charge_response_200 import ChassisChargeResponse200
from ...models.error_envelope import ErrorEnvelope
from ...types import Response


def _get_kwargs() -> dict[str, Any]:

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/chassis/charge",
    }

    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ChassisChargeResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ChassisChargeResponse200.from_dict(response.json())

        return response_200

    if response.status_code == 400:
        response_400 = ErrorEnvelope.from_dict(response.json())

        return response_400

    if response.status_code == 404:
        response_404 = ErrorEnvelope.from_dict(response.json())

        return response_404

    if response.status_code == 409:
        response_409 = ErrorEnvelope.from_dict(response.json())

        return response_409

    if response.status_code == 502:
        response_502 = ErrorEnvelope.from_dict(response.json())

        return response_502

    if response.status_code == 504:
        response_504 = ErrorEnvelope.from_dict(response.json())

        return response_504

    if client.raise_on_unexpected_status:
        raise errors.UnexpectedStatus(response.status_code, response.content)
    else:
        return None


def _build_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> Response[ChassisChargeResponse200 | ErrorEnvelope]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
) -> Response[ChassisChargeResponse200 | ErrorEnvelope]:
    """Start the mobile base's automatic charging routine

     Sends the base to the charging dock configured with `chassis_maps_charging_dock_put`, and answers
    with the charge state read immediately afterwards, so a display starts from a reading rather than
    from an assumption. It does NOT wait for the base to dock: the base then moves its own fields on its
    own schedule -- `work_mode` to `auto_charging` and `charge.dock` from `not_docking` to `docking`,
    then to `docked` once it is on the pile, and finally to `leaving` and back to `not_docking` when it
    drives off. Whether current is actually flowing is a SEPARATE fact reported only by the battery
    packs, as `charge.charging` with `charge.basis` saying where that came from; a base can be `docked`
    with `charging` still null. Poll `GET /v1/chassis/charge` to follow the flow. Preconditions: a
    charging dock must have been configured (the base otherwise refuses with `kind`
    `dock_unconfigured`), the base must not already be charging (`auto_charging`) or be plugged in by
    hand (`manual_charging`), autonomous navigation must be on (`navigation_disabled`), and the physical
    emergency stop must be released. Every one of those is a 409 naming its `kind`. Gated by the chassis
    soft-kill latch.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisChargeResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs()

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    *,
    client: AuthenticatedClient | Client,
) -> ChassisChargeResponse200 | ErrorEnvelope | None:
    """Start the mobile base's automatic charging routine

     Sends the base to the charging dock configured with `chassis_maps_charging_dock_put`, and answers
    with the charge state read immediately afterwards, so a display starts from a reading rather than
    from an assumption. It does NOT wait for the base to dock: the base then moves its own fields on its
    own schedule -- `work_mode` to `auto_charging` and `charge.dock` from `not_docking` to `docking`,
    then to `docked` once it is on the pile, and finally to `leaving` and back to `not_docking` when it
    drives off. Whether current is actually flowing is a SEPARATE fact reported only by the battery
    packs, as `charge.charging` with `charge.basis` saying where that came from; a base can be `docked`
    with `charging` still null. Poll `GET /v1/chassis/charge` to follow the flow. Preconditions: a
    charging dock must have been configured (the base otherwise refuses with `kind`
    `dock_unconfigured`), the base must not already be charging (`auto_charging`) or be plugged in by
    hand (`manual_charging`), autonomous navigation must be on (`navigation_disabled`), and the physical
    emergency stop must be released. Every one of those is a 409 naming its `kind`. Gated by the chassis
    soft-kill latch.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisChargeResponse200 | ErrorEnvelope
    """

    return sync_detailed(
        client=client,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
) -> Response[ChassisChargeResponse200 | ErrorEnvelope]:
    """Start the mobile base's automatic charging routine

     Sends the base to the charging dock configured with `chassis_maps_charging_dock_put`, and answers
    with the charge state read immediately afterwards, so a display starts from a reading rather than
    from an assumption. It does NOT wait for the base to dock: the base then moves its own fields on its
    own schedule -- `work_mode` to `auto_charging` and `charge.dock` from `not_docking` to `docking`,
    then to `docked` once it is on the pile, and finally to `leaving` and back to `not_docking` when it
    drives off. Whether current is actually flowing is a SEPARATE fact reported only by the battery
    packs, as `charge.charging` with `charge.basis` saying where that came from; a base can be `docked`
    with `charging` still null. Poll `GET /v1/chassis/charge` to follow the flow. Preconditions: a
    charging dock must have been configured (the base otherwise refuses with `kind`
    `dock_unconfigured`), the base must not already be charging (`auto_charging`) or be plugged in by
    hand (`manual_charging`), autonomous navigation must be on (`navigation_disabled`), and the physical
    emergency stop must be released. Every one of those is a 409 naming its `kind`. Gated by the chassis
    soft-kill latch.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisChargeResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs()

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
) -> ChassisChargeResponse200 | ErrorEnvelope | None:
    """Start the mobile base's automatic charging routine

     Sends the base to the charging dock configured with `chassis_maps_charging_dock_put`, and answers
    with the charge state read immediately afterwards, so a display starts from a reading rather than
    from an assumption. It does NOT wait for the base to dock: the base then moves its own fields on its
    own schedule -- `work_mode` to `auto_charging` and `charge.dock` from `not_docking` to `docking`,
    then to `docked` once it is on the pile, and finally to `leaving` and back to `not_docking` when it
    drives off. Whether current is actually flowing is a SEPARATE fact reported only by the battery
    packs, as `charge.charging` with `charge.basis` saying where that came from; a base can be `docked`
    with `charging` still null. Poll `GET /v1/chassis/charge` to follow the flow. Preconditions: a
    charging dock must have been configured (the base otherwise refuses with `kind`
    `dock_unconfigured`), the base must not already be charging (`auto_charging`) or be plugged in by
    hand (`manual_charging`), autonomous navigation must be on (`navigation_disabled`), and the physical
    emergency stop must be released. Every one of those is a 409 naming its `kind`. Gated by the chassis
    soft-kill latch.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisChargeResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            client=client,
        )
    ).parsed
